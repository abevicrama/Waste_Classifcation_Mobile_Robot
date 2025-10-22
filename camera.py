# -----------------------------------------------------------------------------
# Pi Camera Preview Script (Requires picamera2 library)
# -----------------------------------------------------------------------------
# This script initializes the Raspberry Pi Camera and displays a live preview.
# Press 'Enter' in the terminal to stop the preview and close the script.
#
# SETUP STEPS:
# 1. Ensure your camera module is correctly connected to the CSI port.
# 2. Install the necessary library:
#    pip install picamera2
# 3. Enable the camera module (often done automatically, but check if needed).
# 4. Run the script: python3 camera_preview.py
# -----------------------------------------------------------------------------

from picamera2 import Picamera2, Preview
import time
import sys

def run_camera_preview():
    """Initializes the camera and starts a live preview window."""
    print("Initializing Picamera2...")

    # Initialize Picamera2
    try:
        # Note: The Pi Zero W might not have enough power/memory for high resolutions,
        # so we rely on the default configuration which is usually stable.
        picam2 = Picamera2()
    except Exception as e:
        print(f"Error initializing camera: {e}")
        print("Please ensure the camera module is connected and the 'picamera2' library is installed.")
        sys.exit(1)

    # Configure the camera for a simple video preview
    # We use a smaller size for the preview window to save resources on the Pi Zero W.
    preview_config = picam2.create_preview_configuration(main={"size": (640, 480)})
    picam2.configure(preview_config)

    # Start the preview. The 'qt' preview is typically used on the desktop (via VNC or SSH X-forwarding).
    # If you are running headless, you may need a different approach or run an HTTP stream.
    print("Starting live preview (640x480)...")
    picam2.start_preview(Preview.QTGL)

    # Start the camera capture
    picam2.start()

    try:
        # Wait until the user presses Enter to stop
        input("Preview is running. Press Enter to stop the camera and exit...")
    except EOFError:
        # Handle case where script is run non-interactively or Ctrl+C is pressed
        print("\nStopping camera due to user intervention.")
    finally:
        # Clean up and stop the camera
        picam2.stop_preview()
        picam2.stop()
        print("Camera stopped. Goodbye.")

if __name__ == "__main__":
    run_camera_preview()
