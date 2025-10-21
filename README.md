# Raspberry Pi Camera Recorder

This small script records video from a Raspberry Pi camera. It prefers Picamera2 (recommended on modern Raspberry Pi OS) and falls back to OpenCV's VideoCapture when Picamera2 isn't available.

Files
- `main.py` — the recorder script (tries Picamera2 first, then OpenCV)
- `requirements.txt` — Python packages that can be installed with pip

Quick start (on Raspberry Pi)

1. Enable the camera and update OS:

   sudo raspi-config  # enable camera interface (if present) or ensure libcamera is enabled
   sudo apt update && sudo apt upgrade -y

2. Install Picamera2 (recommended) and libcamera support (apt is recommended):

   sudo apt install -y python3-picamera2 libcamera-apps

3. (Optional) Install Python dependencies via pip for OpenCV fallback and numpy:

   python3 -m pip install --user -r requirements.txt

Notes
- On Raspberry Pi OS, `picamera2` is often installed via `apt` rather than pip. If you installed it via apt, it won't appear in pip but the script will still detect it.
- If Picamera2 isn't available the script uses OpenCV's `VideoCapture(0)`. That requires libcamera drivers (and often v4l2loopback or similar) or a supported V4L2 driver.

Running

Example commands:

  python3 main.py --duration 10
  python3 main.py --duration 0 --preview
  python3 main.py --output myvideo.mp4 --fps 24

Press Ctrl-C to stop recording if duration is 0 (unlimited). If `--preview` is used you can also press `q` in the preview window to stop early.

Troubleshooting
- If you see "Cannot open camera via OpenCV VideoCapture(0)", ensure the camera is connected and that `libcamera` drivers are functioning.
- If you have issues with Picamera2, check that `python3-picamera2` was installed and the Pi firmware and OS are up-to-date.

Want more features?
- I can add live streaming over the network, upload to a server, or create a small web UI for preview and control. Tell me which you'd like.
