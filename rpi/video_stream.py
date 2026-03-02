from flask import Flask, Response
import cv2
import numpy as np
import threading

app = Flask(__name__)

SMOOTHING_RADIUS = 15

class VideoStabilizer:
    def __init__(self):
        self.prev_gray = None
        self.transforms = []

    def stabilize(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.prev_gray is None:
            self.prev_gray = gray
            return frame

        prev_pts = cv2.goodFeaturesToTrack(
            self.prev_gray, maxCorners=200, qualityLevel=0.01,
            minDistance=30, blockSize=3
        )

        if prev_pts is None or len(prev_pts) < 10:
            self.prev_gray = gray
            return frame

        curr_pts, status, _ = cv2.calcOpticalFlowPyrLK(
            self.prev_gray, gray, prev_pts, None
        )
        idx = np.where(status == 1)[0]
        if len(idx) < 6:
            self.prev_gray = gray
            return frame

        prev_pts = prev_pts[idx]
        curr_pts = curr_pts[idx]

        m, _ = cv2.estimateAffinePartial2D(prev_pts, curr_pts)
        if m is None:
            self.prev_gray = gray
            return frame

        dx = m[0, 2]
        dy = m[1, 2]
        da = np.arctan2(m[1, 0], m[0, 0])

        self.transforms.append([dx, dy, da])

        window = min(len(self.transforms), SMOOTHING_RADIUS)
        recent = np.array(self.transforms[-window:])
        avg = np.mean(recent, axis=0)

        cos_a = np.cos(-avg[2])
        sin_a = np.sin(-avg[2])
        correction = np.array([
            [cos_a, -sin_a, -avg[0]],
            [sin_a,  cos_a, -avg[1]]
        ], dtype=np.float64)

        h, w = frame.shape[:2]
        stabilized = cv2.warpAffine(frame, correction, (w, h),
                                     borderMode=cv2.BORDER_REPLICATE)

        if len(self.transforms) > 300:
            self.transforms = self.transforms[-SMOOTHING_RADIUS:]

        self.prev_gray = gray
        return stabilized


camera = None
lock = threading.Lock()
stabilizer = VideoStabilizer()

def get_camera():
    global camera
    if camera is None or not camera.isOpened():
        # Force V4L2 backend, capture at 1280x720 for wider FOV
        camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        camera.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        camera.set(cv2.CAP_PROP_FPS, 30)
        if not camera.isOpened():
            print("ERROR: Cannot open camera!")
        else:
            w = int(camera.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"Camera opened: {w}x{h}")
    return camera

def generate_frames():
    while True:
        with lock:
            cam = get_camera()
            success, frame = cam.read()
        if not success:
            continue

        frame = stabilizer.stabilize(frame)

        # Encode and stream
        ret, buffer = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if not ret:
            continue
        yield (b"--frame\r\n"
               b"Content-Type: image/jpeg\r\n\r\n" + buffer.tobytes() + b"\r\n")

@app.route("/video_feed")
def video_feed():
    return Response(generate_frames(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/")
def index():
    return "<h1>Robot Camera</h1><img src='/video_feed' width='1280'>"

if __name__ == "__main__":
    print("Wide-angle stabilized video stream on port 8000...")
    app.run(host="0.0.0.0", port=8000, threaded=True)
