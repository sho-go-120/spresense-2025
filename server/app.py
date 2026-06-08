from flask import Flask, request, send_from_directory, render_template
import os
from datetime import datetime

app = Flask(__name__)
UPLOAD_DIR = "static/uploads"
os.makedirs(UPLOAD_DIR, exist_ok=True)

@app.route("/uploads/<filename>")
def uploaded_file(filename):
    return send_from_directory(UPLOAD_DIR, filename)

@app.route("/")
def index():
    files = sorted(os.listdir(UPLOAD_DIR), reverse=True)
    return render_template("index.html", files=files)

@app.route("/upload", methods=["POST"])
def upload():
    data = request.data
    if not data:
        return "No data", 400

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"scream_{ts}.jpg"
    path = os.path.join(UPLOAD_DIR, filename)

    with open(path, "wb") as f:
        f.write(data)

    print("Saved:", filename, "bytes:", len(data))
    return "OK", 200

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8000)
