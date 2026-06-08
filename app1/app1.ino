#include <SDHCI.h>
#include <Camera.h>
#include <Audio.h>
#include <arch/board/board.h>
#include <HttpGs2200.h>
#include <TelitWiFi.h>
#include "configapp1.h"

// =========================
// Audio (Scream detection)
// =========================
#define MIC_GAIN 210
#define SAMPLE_RATE 16000
#define CHANNELS 1
#define FRAME_SIZE 256
#define THRESHOLD 4000  // scream 判定（仮）

AudioClass* theAudio;
int16_t pcm_buf[FRAME_SIZE];

enum AudioMode {
  AUDIO_MONITOR,   // 普段：音量チェック
  AUDIO_IGNORE     // scream～送信中：PCMは読むが判定しない
};

volatile AudioMode audio_mode = AUDIO_MONITOR;

// 送信中などに Audio FIFO が詰まらないように読み捨て
inline void audio_drain() {
  int16_t dummy[FRAME_SIZE];
  uint32_t read_size = 0;
  theAudio->readFrames((char*)dummy, sizeof(dummy), &read_size);
}
// =========================
// Camera (Streaming JPEG)
// =========================
SDClass theSD;

volatile bool g_capture_request = false; // true のとき CamCB が 1枚だけ取る
uint32_t capture_wait_ms = 0;

int picture_id = 0;
uint8_t jpeg_buf[80 * 1024];
size_t jpeg_size = 0;
volatile bool jpeg_ready = false;

bool scream_active = false;

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
  audio_drain();
  cid = gs2200.connect(HTTP_SRVR_IP, HTTP_PORT);
  audio_drain();
  if (cid == ATCMD_INVALID_CID) {
    Serial.println("[RAW] connect failed");
    return false;
  }
  audio_drain();
  WiFi_InitESCBuffer();
  audio_drain();
  String header =
    String("POST ") + HTTP_POST_PATH + " HTTP/1.1\r\n" +
    "Host: " + String(HTTP_SRVR_IP) + ":" + String(HTTP_PORT) + "\r\n" +
    "Content-Type: image/jpeg\r\n" +
    "Content-Length: " + String(len) + "\r\n" +
    "Connection: close\r\n" +
    "\r\n";

  if (!gs2200.write(cid, (const uint8_t*)header.c_str(), header.length())) {
    Serial.println("[RAW] header write failed");
    audio_drain();
    gs2200.stop(cid);
    delay(10);
    return false;
  }

  // --- body ---
  size_t sent = 0;
  const size_t CHUNK = 256;
  int retry = 0;

  while (sent < len) {
    audio_drain();  // 送信中も Audio FIFO を捨てて詰まり回避

    size_t n = min(CHUNK, len - sent);
    if (!gs2200.write(cid, data + sent, n)) {
      retry++;
      audio_drain();
      if (retry > 50) {
        audio_drain();
        Serial.println("[RAW] body write abort");
        gs2200.stop(cid);
        delay(10);
        return false;
      }
      delay(10);
      continue;
    }
    retry = 0;
    sent += n;
    delay(10);
  }

  Serial.println("[RAW] body sent, waiting response...");

  // --- response ---
  uint8_t rx[256];
  uint32_t t0 = millis();
  while (millis() - t0 < 2000) {
    audio_drain(); // レスポンス待ち中も捨てる

    if (gs2200.available()) {
      int r = gs2200.read(cid, rx, sizeof(rx) - 1);
      if (r > 0) {
        rx[r] = 0;
        Serial.print((char*)rx);

        if (strstr((char*)rx, "200 OK") || strstr((char*)rx, "HTTP/1.1 200")) {
          Serial.println("[RAW] HTTP 200 OK");
          gs2200.stop(cid);
          delay(10);
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
// Camera stream callback (1枚だけ)
// -------------------------
void CamCB(CamImage img) {
  if (!img.isAvailable()) return;
  if (!g_capture_request) return;
  if (jpeg_ready) return; // 未処理が残ってたら取らない

  jpeg_size = img.getImgSize();
  memcpy(jpeg_buf, img.getImgBuff(), jpeg_size);
  audio_drain();
  jpeg_ready = true;
  g_capture_request = false; // 1枚だけ撮影

  Serial.println("CamCB: JPEG captured");
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

  // ---- Camera init ----
  Serial.println("Camera begin...");
  CamErr err = theCamera.begin(
    2,
    CAM_VIDEO_FPS_30,
    CAM_IMGSIZE_QVGA_H,
    CAM_IMGSIZE_QVGA_V,
    CAM_IMAGE_PIX_FMT_JPG
  );
  if (err != CAM_ERR_SUCCESS) {
    Serial.print("Camera begin failed: ");
    Serial.println((int)err);
    while (1);
  }

  theCamera.setHDR(CAM_HDR_MODE_ON);
  theCamera.setAutoExposure(true);
  theCamera.setAutoWhiteBalance(true);
  theCamera.setAutoWhiteBalanceMode(CAM_WHITE_BALANCE_AUTO);
  theCamera.setAutoISOSensitivity(false);
  theCamera.setISOSensitivity(1250 * 1000);
  theCamera.setJPEGQuality(60);

  Serial.println("Camera startStreaming...");
  err = theCamera.startStreaming(true, CamCB);
  if (err != CAM_ERR_SUCCESS) {
    Serial.print("startStreaming failed: ");
    Serial.println((int)err);
    while (1);
  }
  Serial.println("Camera streaming OK");

  // ---- WiFi init ----
  Init_GS2200_SPI_type(iS110B_TypeC);
  gsparams.mode = ATCMD_MODE_STATION;
  gsparams.psave = ATCMD_PSAVE_DEFAULT;

  if (gs2200.begin(gsparams)) {
    Serial.println("GS2200 begin failed");
    while (1);
  }
  if (gs2200.activate_station(AP_SSID, PASSPHRASE)) {
    Serial.println("WiFi connect failed");
    while (1);
  }
  Serial.println("WiFi connected");

  // ---- HTTP init ----
  hostParams.host = (char*)HTTP_SRVR_IP;
  hostParams.port = (char*)HTTP_PORT;
  theHttpGs2200.begin(&hostParams);
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
  // --- 1) 送信・保存処理（jpeg_ready 優先）
  if (jpeg_ready) {
    audio_mode = AUDIO_IGNORE; // 送信中は判定しない（読むだけ）

    char filename[32];
    sprintf(filename, "SCREAM_%04d.JPG", picture_id++);

    File f = theSD.open(filename, FILE_WRITE);
    if (!f) {
      Serial.println("SD open failed");
      jpeg_ready = false;
      scream_active = false;
      audio_mode = AUDIO_MONITOR;
      return;
    }

    audio_drain();
    f.write(jpeg_buf, jpeg_size);
    audio_drain();
    f.close();
    audio_drain();

    Serial.print("Saved: ");
    Serial.println(filename);

    Serial.println("POST JPEG (raw)...");
    bool ok = post_jpeg_raw(jpeg_buf, jpeg_size);

    if (ok) {
      Serial.println("POST OK -> remove SD");
      theSD.remove(filename);
    } else {
      Serial.println("POST FAIL (keep SD)");
    }

    jpeg_ready = false;
    scream_active = false;
    audio_mode = AUDIO_MONITOR;
    digitalWrite(LED0, LOW);
    return;
  }

  // --- 2) capture timeout（要求中だけ）
  if (g_capture_request && (millis() - capture_wait_ms > 10000)) {
    audio_drain();
    Serial.println("Capture timeout -> cancel");
    g_capture_request = false;
    scream_active = false;
    audio_mode = AUDIO_MONITOR;
    digitalWrite(LED0, LOW);
  }

  // --- 3) Audio フレーム読み（常に読む：止めない）
  uint32_t total_read = 0;
  while (total_read < FRAME_SIZE) {
    uint32_t read_size = 0;
    int ret = theAudio->readFrames(
      (char*)(pcm_buf + total_read),
      (FRAME_SIZE - total_read) * sizeof(int16_t),
      &read_size
    );

    if (ret != AUDIOLIB_ECODE_OK &&
        ret != AUDIOLIB_ECODE_INSUFFICIENT_BUFFER_AREA) {
      Serial.println("readFrames error");
      return;
    }

    if (read_size > 0) {
      total_read += read_size / sizeof(int16_t);
    } else {
      delay(1);
    }
  }

  // --- 4) 判定（MONITOR の時だけ）
  if (audio_mode == AUDIO_MONITOR) {
    int16_t max_val = 0;
    for (int i = 0; i < FRAME_SIZE; i++) {
      int16_t v = abs(pcm_buf[i]);
      if (v > max_val) max_val = v;
    }

    Serial.println(max_val);

    if (max_val > THRESHOLD && !scream_active && !g_capture_request) {
      scream_active = true;
      audio_mode = AUDIO_IGNORE;

      digitalWrite(LED0, HIGH);
      Serial.println("SCREAM!");
      Serial.println("Capture requested");
      audio_drain();
      g_capture_request = true;
      capture_wait_ms = millis();   // timeout用
    }
  }
}
