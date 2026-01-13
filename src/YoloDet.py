import cv2
import torch
import sys
import os

from GeometryUtils import draw_lines_and_compute_angle


FILE = os.path.dirname(os.path.abspath(__file__))
yolov5_path = os.path.join(FILE, "yolov5")
sys.path.append(yolov5_path)

model_path = "/home/juhdi/catkin_ws/src/skripsi/data/bestv5.pt"
video_path = "/home/juhdi/catkin_ws/src/skripsi/data/ballYolo.mp4"

xt = 120
yt = 120
centers = {}

model = torch.hub.load(
    yolov5_path,
    'custom',
    path=model_path,
    source='local'
)

model.conf = 0.25

cap = cv2.VideoCapture(video_path)

if not cap.isOpened():
    print("Gagal membuka video:", video_path)
    exit()


while True:
    ret, frame = cap.read()
    if not ret:
        print("Video selesai.")
        break

    results = model(frame)
    detections = results.xyxy[0]
    if detections is not None and len(detections) > 0:
        detections = detections.cpu().numpy()
        largest_boxes = {}

        for det in detections:
            x1, y1, x2, y2, conf, cls = det
            cls = int(cls)

            area = (x2 - x1) * (y2 - y1)
            if cls not in largest_boxes or area > largest_boxes[cls]["area"]:
                largest_boxes[cls] = {
                    "bbox": (x1, y1, x2, y2),
                    "area": area,
                    "conf": conf
                }

        for cls, data in largest_boxes.items():
            x1, y1, x2, y2 = data["bbox"]
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            print(f"Class {cls} | Center: ({cx}, {cy}) | Conf: {data['conf']:.2f}")
            centers[cls] = (cx, cy)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
    
        angle = draw_lines_and_compute_angle(
            frame,
            p1=(centers[0]),
            p2=(centers[1]),
            target_point=(xt, yt)
        )

    results.xyxy[0] = torch.tensor(
        [list(v["bbox"]) + [v["conf"], cls]
         for cls, v in largest_boxes.items()]
    ) if 'largest_boxes' in locals() else results.xyxy[0]

    

    output = results.render()[0]
    cv2.imshow("YOLOv5 Detection", output)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()