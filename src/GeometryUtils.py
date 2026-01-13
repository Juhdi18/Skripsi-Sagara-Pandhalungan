import cv2
import math

def draw_lines_and_compute_angle(
    frame,
    p1,
    p2,
    target_point,
    color_main=(0, 255, 0),
    color_target=(0, 0, 255),
    color_center=(255, 0, 0),
    thickness=2
):
    # =========================
    # CAST SEMUA KE INTEGER
    # =========================
    x1, y1 = int(p1[0]), int(p1[1])
    x2, y2 = int(p2[0]), int(p2[1])
    xt, yt = int(target_point[0]), int(target_point[1])

    # =========================
    # GARIS PERTAMA (PAKAI INT!)
    # =========================
    cv2.line(frame, (x1, y1), (x2, y2), color_main, thickness)

    # =========================
    # TITIK TENGAH
    # =========================
    cx = int((x1 + x2) / 2)
    cy = int((y1 + y2) / 2)
    cv2.circle(frame, (cx, cy), 5, color_main, -1)

    # =========================
    # GARIS KEDUA
    # =========================
    cv2.line(frame, (cx, cy), (xt, yt), color_target, thickness)
    cv2.circle(frame, (xt, yt), 5, color_target, -1)

    # =========================
    # GARIS TENGAH KAMERA
    # =========================
    h, w = frame.shape[:2]
    cam_center_x = w // 2
    cv2.line(frame, (cam_center_x, 0), (cam_center_x, h), color_center, 1)

    # =========================
    # HITUNG SUDUT
    # =========================
    v1x, v1y = 0, -1
    v2x = xt - cx
    v2y = cy - yt

    dot = v1x * v2x + v1y * v2y
    mag1 = math.sqrt(v1x**2 + v1y**2)
    mag2 = math.sqrt(v2x**2 + v2y**2)

    if mag2 == 0:
        angle_deg = 0.0
    else:
        angle_rad = math.acos(dot / (mag1 * mag2))
        angle_deg = math.degrees(angle_rad)

        if xt < cam_center_x:
            angle_deg = -angle_deg

    cv2.putText(
        frame,
        f"Angle: {angle_deg:.2f} deg",
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    return cx, cy, angle_deg
