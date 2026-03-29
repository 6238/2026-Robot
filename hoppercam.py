import argparse
import cv2
import numpy as np
from http.server import BaseHTTPRequestHandler, HTTPServer
import threading
import time

# --- Changed Import ---
from networktables import NetworkTables 


def parse_args():
    p = argparse.ArgumentParser(
        description="Hopper fill level estimation via color mask + pynetworktables."
    )
    p.add_argument("-c", "--camera", type=int, default=0, help="Camera index")
    p.add_argument("--hue-low", type=int, default=20)
    p.add_argument("--hue-high", type=int, default=35)
    p.add_argument("--sat-low", type=int, default=100)
    p.add_argument("--sat-high", type=int, default=255)
    p.add_argument("--val-low", type=int, default=100)
    p.add_argument("--val-high", type=int, default=255)
    p.add_argument("--team", type=int, default=6238, help="FRC team number")
    p.add_argument("--stream-port", type=int, default=5900)
    p.add_argument("--no-stream", action="store_true")
    return p.parse_args()


# --- Shared state for MJPEG stream ---
latest_frame: bytes = b""
frame_lock = threading.Lock()

class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, _format, *_args): pass
    def do_GET(self):
        if self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with frame_lock: frame = latest_frame
                    if frame:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + frame + b"\r\n")
                    time.sleep(0.05)
            except (BrokenPipeError, ConnectionResetError): pass
        else:
            self.send_response(404)
            self.end_headers()

def run_server(port: int):
    server = HTTPServer(("0.0.0.0", port), MJPEGHandler)
    server.serve_forever()

def compute_fill(frame: np.ndarray, lower: np.ndarray, upper: np.ndarray):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    raw = int(np.count_nonzero(mask)) / mask.size
    return raw, raw**2, mask


def main():
    args = parse_args()

    color_lower = np.array([args.hue_low, args.sat_low, args.val_low])
    color_upper = np.array([args.hue_high, args.sat_high, args.val_high])
    enable_stream = not args.no_stream

    # --- pynetworktables setup ---
    # Construct the standard FRC IP: 10.TE.AM.2
    team_ip = f"10.{args.team // 100}.{args.team % 100}.2"
    NetworkTables.initialize(server=team_ip)
    table = NetworkTables.getTable("hoppercam")

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"ERROR: Could not open camera {args.camera}")
        return

    if enable_stream:
        threading.Thread(target=run_server, args=(args.stream_port,), daemon=True).start()
        print(f"Stream available at http://0.0.0.0:{args.stream_port}/stream")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.1)
                continue

            raw_fill, corrected_fill, mask = compute_fill(frame, color_lower, color_upper)

            # --- pynetworktables Publishing ---
            table.putNumber("fill", corrected_fill)
            table.putNumber("mask_pixels", float(np.count_nonzero(mask)))

            if enable_stream:
                mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
                debug = np.hstack([frame, mask_bgr])
                cv2.putText(debug, f"raw={raw_fill:.3f} fill={corrected_fill:.3f}", 
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                _, jpeg = cv2.imencode(".jpg", debug)
                with frame_lock:
                    global latest_frame
                    latest_frame = jpeg.tobytes()

            time.sleep(0.033) 
    except KeyboardInterrupt:
        print("Stopping.")
    finally:
        cap.release()

if __name__ == "__main__":
    main()