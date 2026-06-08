#include <SDHCI.h>
#include <Camera.h>
#include <Audio.h>
#include <arch/board/board.h>
#include <HttpGs2200.h>
#include <TelitWiFi.h>
#include "configapp.h"

// =========================
// Audio (Scream detection)
// =========================
#define MIC_GAIN 210
#define SAMPLE_RATE 16000
#define CHANNELS 1
#define FRAME_SIZE 256
#define THRESHOLD 4000  // 叫び判定（仮）
AudioClass* theAudio;
int16_t pcm_buf[FRAME_SIZE];
enum AudioMode {
  AUDIO_MONITOR,   // 普段：音量チェック
  AUDIO_IGNORE     // scream中：PCMは読むが捨てる
};

volatile AudioMode audio_mode = AUDIO_MONITOR;
inline void audio_drain() {
  int16_t dummy[FRAME_SIZE];
  uint32_t read_size;
  theAudio->readFrames(
    (char*)dummy,
    sizeof(dummy),
    &read_size
  );
}
// =========================
// Camera (Streaming JPEG)
// =========================
SDClass theSD;
volatile bool g_capture_request = false;  // scream検知でtrueにして、CamCBで消費
// volatile bool g_saving = false;          // SD書き込み中のガード
int picture_id = 0;
uint8_t jpeg_buf[80 * 1024];  // QVGA JPEGなら十分
size_t jpeg_size = 0;
volatile bool jpeg_ready = false;
#define BURST_COUNT 3
volatile int capture_remain = 0;

// ---- Silence detection ----
#define SILENCE_FRAMES 15  // 無音が続いたら scream 終了
uint16_t silent_count = 0;
bool scream_active = false;
uint32_t capture_wait_ms = 0;

// =========================
// HTTP
// =========================
TelitWiFi gs2200;
TWIFI_Params gsparams;
HttpGs2200 theHttpGs2200(&gs2200);
HTTPGS2200_HostParams hostParams;

// -------------------------
// Audio error callback
// -------------------------
static void audio_attention_cb(const ErrorAttentionParam* atprm) {
  if (atprm->error_code >= AS_ATTENTION_CODE_WARNING) {
    Serial.println("Audio Error!");
  }
}

// =========================
// RAW HTTP POST over TCP (binary JPEG)
// =========================
static bool post_jpeg_raw(const uint8_t* data, size_t len) {
  char cid = ATCMD_INVALID_CID;

  Serial.println("[RAW] connect TCP...");
  cid = gs2200.connect(HTTP_SRVR_IP, HTTP_PORT);

  if (cid == ATCMD_INVALID_CID) {
    Serial.println("[RAW] connect failed");
    return false;
  }

  WiFi_InitESCBuffer();

  // --- HTTP header ---
  String header =
    String("POST ") + HTTP_POST_PATH + " HTTP/1.1\r\n" + "Host: " + String(HTTP_SRVR_IP) + ":" + String(HTTP_PORT) + "\r\n" + "Content-Type: image/jpeg\r\n" + "Content-Length: " + String(len) + "\r\n" + "Connection: close\r\n" + "\r\n";

  // ヘッダ送信
  if (!gs2200.write(cid, (const uint8_t*)header.c_str(), header.length())) {
    Serial.println("[RAW] header write failed");
    gs2200.stop(cid);
    delay(10);
    return false;
  }

  // --- body ---
  size_t sent = 0;
  const size_t CHUNK = 256;
  int retry = 0;
  while (sent < len) {
    audio_drain(); 

    size_t n = min(CHUNK, len - sent);
    if (!gs2200.write(cid, data + sent, n)) {
      retry++;
      if (retry > 30) {  // retry制限(次の写真を優先)
        Serial.println("[RAW] body write abort");
        gs2200.stop(cid);
        delay(50);
        return false;
      }
      delay(20);
      continue;
    }
    retry = 0;
    sent += n;
    delay(20);
  }

  Serial.println("[RAW] body sent, waiting response...");

  // --- response ---
  uint8_t rx[256];
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    audio_drain();
    if (gs2200.available()) {
      int r = gs2200.read(cid, rx, sizeof(rx) - 1);
      if (r > 0) {
        rx[r] = 0;
        Serial.print((char*)rx);

        if (strstr((char*)rx, "200 OK") || strstr((char*)rx, "HTTP/1.1 200")) {
          audio_drain();
          Serial.println("[RAW] HTTP 200 OK");
          gs2200.stop(cid);
          delay(10);
          audio_drain();
          return true;
        }
      }
    }
    delay(10);
  }

  Serial.println("[RAW] no response / timeout");
  gs2200.stop(cid);
  delay(10);
  return false;
}


// -------------------------
// Camera stream callback
// -------------------------
void CamCB(CamImage img) {
  if (!img.isAvailable()) return;
  if (capture_remain <= 0) return;
  if (!g_capture_request) return;
  if (jpeg_ready) return;

  jpeg_size = img.getImgSize();
  memcpy(jpeg_buf, img.getImgBuff(), jpeg_size);

  jpeg_ready = true;
  capture_remain--;

  Serial.print("CamCB: JPEG captured, remain=");
  Serial.println(capture_remain);

  if (capture_remain <= 0) {
    g_capture_request = false;
  }
}
// =========================
// setup
// =========================
void setup() {
  Serial.begin(115200);
  pinMode(LED0, OUTPUT);
  digitalWrite(LED0, LOW);

  // ---- SD init ----
  while (!theSD.begin()) {
    Serial.println("Insert SD card.");
    delay(500);
  }
  Serial.println("SD OK");

  // ---- Camera init (Streaming JPEG) ----
  Serial.println("Camera begin...");

  CamErr err;

  // JPEGストリーム（QVGA/30fps）で軽量・安定を狙う
  err = theCamera.begin(
    2,  // buffer count (2以上推奨)
    CAM_VIDEO_FPS_30,
    CAM_IMGSIZE_QVGA_H,
    CAM_IMGSIZE_QVGA_V,
    CAM_IMAGE_PIX_FMT_JPG);
  if (err != CAM_ERR_SUCCESS) {
    Serial.print("Camera begin failed: ");
    Serial.println((int)err);
    while (1)
      ;
  }

  // お化け屋敷向けプリセット（暗所＋急な照明変化）
  theCamera.setHDR(CAM_HDR_MODE_ON);
  theCamera.setAutoExposure(true);
  theCamera.setAutoWhiteBalance(true);
  theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_AUTO);
  theCamera.setAutoISOSensitivity(false);    // ISOを上げて暗所を優先（ノイズは許容）
  theCamera.setISOSensitivity(1250 * 1000);  // ISO 1250
  theCamera.setJPEGQuality(60);              // JPEG品質（暗所ノイズが多いので高くしすぎない）

  Serial.println("Camera startStreaming...");
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS) {
    Serial.print("startStreaming failed: ");
    Serial.println((int)err);
    while (1)
      ;
  }
  Serial.println("Camera streaming OK");

  // ---- WiFi (GS2200) init ----
  Init_GS2200_SPI_type(iS110B_TypeC);
  gsparams.mode = ATCMD_MODE_STATION;
  gsparams.psave = ATCMD_PSAVE_DEFAULT;

  if (gs2200.begin(gsparams)) {
    Serial.println("GS2200 begin failed");
    while (1)
      ;
  }

  if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
    Serial.println("WiFi connect failed");
    while (1)
      ;
  }

  Serial.println("WiFi connected");

  // ---- HTTP init ----
  hostParams.host = (char*)HTTP_SRVR_IP;
  hostParams.port = (char*)HTTP_PORT;
  theHttpGs2200.begin(&hostParams);
  // 必須ヘッダ
  theHttpGs2200.config(HTTP_HEADER_HOST, HTTP_SRVR_IP);
  theHttpGs2200.config(HTTP_HEADER_CONTENT_TYPE, "text/plain");
  theHttpGs2200.config(HTTP_HEADER_TRANSFER_ENCODING, "identity");

  Serial.println("HTTP client ready");

  // ---- Audio init ----
  theAudio = AudioClass::getInstance();
  theAudio->begin(audio_attention_cb);
  theAudio->setRecorderMode(AS_SETRECDR_STS_INPUTDEVICE_MIC, MIC_GAIN);
  theAudio->initRecorder(AS_CODECTYPE_WAV, "/mnt/sd0/BIN", SAMPLE_RATE, CHANNELS);
  theAudio->startRecorder();

  Serial.println("Audio ready");
  Serial.println("SCREAM detector started");
}

// =========================
// loop
// =========================
void loop() {
  // CamCB が来なかった保険(キャプチャ中のみ)
  if (g_capture_request && !jpeg_ready && (millis() - capture_wait_ms > 6000)) {
    Serial.println("Capture timeout -> restart audio");
    scream_active = false;
    silent_count = 0;
    g_capture_request = false;
    capture_remain = 0;
    audio_mode = AUDIO_MONITOR;
    return;
  }
  if (jpeg_ready) {
    char filename[32];
    sprintf(filename, "SCREAM_%04d_%d.JPG", picture_id++, BURST_COUNT-capture_remain);

    File f = theSD.open(filename, FILE_WRITE);
    if (f) {
      audio_drain(); 
      f.write(jpeg_buf, jpeg_size);
      f.close();
      audio_drain(); 
      Serial.print("Saved: ");
      Serial.println(filename);
    } else {
      Serial.println("SD open failed");
      jpeg_ready = false;

      // 次を拾うための状態リセット
      scream_active = false;
      silent_count = 0;
      return;
    }

    // 生JPEGをHTTP POST（RAW TCP）
    Serial.println("POST JPEG (raw)...");
    bool ok = post_jpeg_raw(jpeg_buf, jpeg_size);

    if (ok) {
      Serial.println("POST OK -> remove SD");
      theSD.remove(filename);
    } else {
      jpeg_ready = false;
      g_capture_request = false;
      capture_remain = 0;

      scream_active = false;
      silent_count = 0;
      audio_mode = AUDIO_MONITOR;

      delay(100);

      Serial.println("Audio force restarted");
      return;
    }

    jpeg_ready = false;   // 次のCamCBを受ける

    // まだ連写が残ってるなら、audioは再開しない
    static uint32_t last = 0;
    if (capture_remain > 0 && millis() - last > 500) {
      audio_drain();
      Serial.print("WAIT... remain=");
      Serial.println(capture_remain);
      delay(50);          // ほんの少しだけ待つ（CamCBが来やすい）
      audio_drain();
      return;
    }

    // 3枚撮り終わったら、ここで初めてaudio復帰＋状態リセット
    audio_drain();
    delay(100);
    Serial.println("Burst done -> resume monitor");

    scream_active = false;
    silent_count = 0;
    g_capture_request = false;
    capture_remain = 0;
    audio_mode = AUDIO_MONITOR;
  }

  // ---- PCM frame get ----
  uint32_t total_read = 0;
  while (total_read < FRAME_SIZE) {
    uint32_t read_size = 0;
    int ret = theAudio->readFrames((char*)(pcm_buf + total_read), (FRAME_SIZE - total_read) * sizeof(int16_t), &read_size);

    if (ret != AUDIOLIB_ECODE_OK && ret != AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA) {
      Serial.println("readFrames error");
      return;
    }

    if (read_size > 0) {
      total_read += read_size / sizeof(int16_t);
    } else {
      delay(1);
    }
  }



  if (audio_mode == AUDIO_MONITOR) {
    int16_t max_val = 0;
    for (int i = 0; i < FRAME_SIZE; i++) {
      int16_t v = abs(pcm_buf[i]);
      if (v > max_val) max_val = v;
    }

    Serial.println(max_val);

    if (max_val > THRESHOLD && !scream_active) {
      scream_active = true;
      audio_mode = AUDIO_IGNORE;
      digitalWrite(LED0, HIGH);
      Serial.println("SCREAM!");

      capture_remain = BURST_COUNT;
      g_capture_request = true;
      capture_wait_ms = millis();

      Serial.println("Capture requested");
      audio_drain();
      delay(100);
    }
  }
}
