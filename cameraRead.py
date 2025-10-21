"""
Record video from a Raspberry Pi camera (Picamera2) or fallback to OpenCV V4L2.

Usage examples (on the Pi):
  python main.py --duration 10                     # record 10 seconds to timestamped MP4
  python main.py --duration 0 --preview            # preview until Ctrl-C (duration 0 means unlimited)
  python main.py --output myvideo.mp4 --fps 30

Notes:
  - Picamera2 (python3-picamera2) is recommended for modern Raspberry Pi OS.
  - If Picamera2 isn't available the script will try OpenCV's VideoCapture(0).
  - See README.md for install hints.
"""

from __future__ import annotations

import argparse
import datetime
import sys
import time
from pathlib import Path

try:
	import cv2
except Exception as e:
	print("OpenCV (cv2) is required. Install with: pip install opencv-python")
	raise

HAS_PICAMERA2 = False
try:
	# Picamera2 is optional but recommended on Raspberry Pi
	from picamera2 import Picamera2
	HAS_PICAMERA2 = True
except Exception:
	HAS_PICAMERA2 = False


def timestamped_filename(prefix: str = "video", ext: str = "mp4") -> str:
	t = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
	return f"{prefix}_{t}.{ext}"


def record_with_opencv(output: str, duration: float, width: int, height: int, fps: int, preview: bool):
	cap = cv2.VideoCapture(0)
	if not cap.isOpened():
		raise RuntimeError("Cannot open camera via OpenCV VideoCapture(0). Is the camera enabled and available?")

	# Try to set resolution and fps (driver may ignore)
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
	cap.set(cv2.CAP_PROP_FPS, fps)

	fourcc = cv2.VideoWriter_fourcc(*"mp4v")
	out = cv2.VideoWriter(output, fourcc, fps, (width, height))

	print(f"Recording (OpenCV) -> {output}  {width}x{height}@{fps}fps")
	start = time.time()
	try:
		while True:
			ret, frame = cap.read()
			if not ret:
				print("Frame read failed, stopping")
				break
			# Some cameras may give different sized frames; resize to requested size for the writer
			if frame.shape[1] != width or frame.shape[0] != height:
				frame = cv2.resize(frame, (width, height))

			out.write(frame)
			if preview:
				cv2.imshow("Preview", frame)
				# Press q to stop preview
				if cv2.waitKey(1) & 0xFF == ord('q'):
					print("Preview requested quit (q)")
					break

			if duration > 0 and (time.time() - start) >= duration:
				break
	except KeyboardInterrupt:
		print("Interrupted by user")
	finally:
		cap.release()
		out.release()
		if preview:
			cv2.destroyAllWindows()


def record_with_picamera2(output: str, duration: float, width: int, height: int, fps: int, preview: bool):
	# This uses Picamera2 to capture numpy arrays and writes them via OpenCV VideoWriter.
	picam2 = Picamera2()

	# Create a simple video configuration sized to the requested resolution
	try:
		video_config = picam2.create_video_configuration(main={"size": (width, height)})
		picam2.configure(video_config)
	except Exception:
		# Fallback to preview/video configuration creation differences across picamera2 versions
		try:
			cfg = picam2.create_preview_configuration({"main":{"size":(width, height)}})
			picam2.configure(cfg)
		except Exception:
			# Try default configure
			picam2.configure()

	picam2.start()

	fourcc = cv2.VideoWriter_fourcc(*"mp4v")
	out = cv2.VideoWriter(output, fourcc, fps, (width, height))

	print(f"Recording (Picamera2) -> {output}  {width}x{height}@{fps}fps")
	start = time.time()
	try:
		while True:
			frame = picam2.capture_array()
			# Picamera2 returns RGB by default; convert to BGR for OpenCV
			frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

			if frame_bgr.shape[1] != width or frame_bgr.shape[0] != height:
				frame_bgr = cv2.resize(frame_bgr, (width, height))

			out.write(frame_bgr)
			if preview:
				cv2.imshow("Preview", frame_bgr)
				if cv2.waitKey(1) & 0xFF == ord('q'):
					print("Preview requested quit (q)")
					break

			if duration > 0 and (time.time() - start) >= duration:
				break
	except KeyboardInterrupt:
		print("Interrupted by user")
	finally:
		out.release()
		picam2.stop()
		if preview:
			cv2.destroyAllWindows()


def main():
	parser = argparse.ArgumentParser(description="Record video from Raspberry Pi camera (Picamera2 or OpenCV)")
	parser.add_argument("--output", "-o", help="Output filename (mp4). If omitted a timestamped file is used.", default=None)
	parser.add_argument("--duration", "-d", type=float, help="Duration in seconds (0 = unlimited until Ctrl-C)", default=10)
	parser.add_argument("--width", type=int, default=640)
	parser.add_argument("--height", type=int, default=480)
	parser.add_argument("--fps", type=int, default=30)
	parser.add_argument("--preview", action="store_true", help="Show a live preview window while recording")
	parser.add_argument("--force-opencv", action="store_true", help="Force using OpenCV even if Picamera2 is available")

	args = parser.parse_args()

	out = args.output or timestamped_filename()
	out_path = Path(out)
	# Ensure parent exists
	if not out_path.parent.exists():
		out_path.parent.mkdir(parents=True, exist_ok=True)

	try:
		if HAS_PICAMERA2 and not args.force_opencv:
			record_with_picamera2(str(out_path), args.duration, args.width, args.height, args.fps, args.preview)
		else:
			record_with_opencv(str(out_path), args.duration, args.width, args.height, args.fps, args.preview)
	except Exception as e:
		print("Recording failed:", e)
		# If Picamera2 attempt failed and we haven't tried OpenCV fallback yet, try it
		if HAS_PICAMERA2 and not args.force_opencv:
			print("Attempting fallback using OpenCV VideoCapture(0)...")
			try:
				record_with_opencv(str(out_path), args.duration, args.width, args.height, args.fps, args.preview)
			except Exception as e2:
				print("Fallback also failed:", e2)
				sys.exit(1)
		else:
			sys.exit(1)


if __name__ == "__main__":
	main()
