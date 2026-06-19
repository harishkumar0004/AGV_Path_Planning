import math
import time
import serial
import cv2
import numpy as np

from perception.camera_manager import CameraManager
from perception.apriltag_detector import AprilTagDetector


SERIAL_PORT = "/dev/ttyUSB0"   # change to /dev/ttyACM0 if needed
BAUD_RATE = 115200

SEND_ALIGN_ON_AT_START = False   # keep False first for safety
SEND_RATE_HZ = 15.0              # do not spam ESP32 too fast
LOST_DELAY_SEC = 0.30            # wait 300 ms before declaring lost
SEND_TAGPOSE_TO_ESP32 = False

TAG_SIZE_M = 0.020
# Temporary approximate intrinsics for Raspberry Pi Camera Module v2.
# Replace these with calibrated camera_matrix and dist_coeffs later.
APPROX_FX = 700.0
APPROX_FY = 700.0
DIST_COEFFS = np.zeros((5, 1), dtype=np.float32)
intrinsics_printed = False
TAG_OBJECT_POINTS = np.array(
    [
        [-TAG_SIZE_M / 2.0, -TAG_SIZE_M / 2.0, 0.0],
        [ TAG_SIZE_M / 2.0, -TAG_SIZE_M / 2.0, 0.0],
        [ TAG_SIZE_M / 2.0,  TAG_SIZE_M / 2.0, 0.0],
        [-TAG_SIZE_M / 2.0,  TAG_SIZE_M / 2.0, 0.0],
    ],
    dtype=np.float32,
)


def calculate_tag_area(corners):
    pts = np.array(corners, dtype=np.float32)
    return abs(cv2.contourArea(pts))


def choose_best_detection(detections):
    if not detections:
        return None

    # Pick largest visible tag area. Good for floor tag when only one should matter.
    return max(detections, key=lambda d: calculate_tag_area(d["corners"]))


def build_camera_matrix(frame_width, frame_height):
    # Temporary approximate intrinsics. Calibrate the camera for real pose use.
    cx = frame_width / 2.0
    cy = frame_height / 2.0
    return np.array(
        [[APPROX_FX, 0.0, cx], [0.0, APPROX_FY, cy], [0.0, 0.0, 1.0]],
        dtype=np.float32,
    )


def print_camera_intrinsics_once(frame_width, frame_height, camera_matrix):
    global intrinsics_printed

    if intrinsics_printed:
        return

    intrinsics_printed = True
    print(
        "CAMERA_INTRINSICS "
        f"fx={camera_matrix[0, 0]:.2f} fy={camera_matrix[1, 1]:.2f} "
        f"cx={camera_matrix[0, 2]:.2f} cy={camera_matrix[1, 2]:.2f} "
        f"frame_width={frame_width} frame_height={frame_height}"
    )


def estimate_tag_pose(detection, frame_width, frame_height):
    image_points = np.array(detection["corners"], dtype=np.float32)
    object_points = np.array(TAG_OBJECT_POINTS, dtype=np.float32)
    camera_matrix = build_camera_matrix(frame_width, frame_height)
    print_camera_intrinsics_once(frame_width, frame_height, camera_matrix)

    try:
        retval, rvec, tvec = cv2.solvePnP(
            object_points,
            image_points,
            camera_matrix,
            DIST_COEFFS,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
    except cv2.error as error:
        print(f"TAGPOSE solvePnP failed for id={detection['tag_id']}: {error}")
        return None

    if not retval:
        return None

    rotation_matrix, _ = cv2.Rodrigues(rvec)
    yaw_deg = math.degrees(math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0]))
    z_m = float(tvec[2][0])

    return {
        "x_m": float(tvec[0][0]),
        "y_m": float(tvec[1][0]),
        "z_m": z_m,
        "yaw_deg": float(yaw_deg),
        "valid": 0.05 <= z_m <= 0.25,
    }


def add_detection_measurements(detection, image_center_x, image_center_y, frame_width, frame_height):
    x_error_px = detection["center_x"] - image_center_x
    y_error_px = detection["center_y"] - image_center_y

    detection["x_norm"] = x_error_px / image_center_x
    detection["y_norm"] = y_error_px / image_center_y

    theta_deg = detection.get("orientation_deg")
    if theta_deg is None:
        theta_deg = 0.0
    detection["theta_deg"] = theta_deg
    detection["pose"] = estimate_tag_pose(detection, frame_width, frame_height)


def print_detection_log(detection):
    tag_id = detection["tag_id"]
    x_norm = detection["x_norm"]
    y_norm = detection["y_norm"]
    theta_deg = detection["theta_deg"]

    print(f"TAG     {tag_id} {x_norm:.4f} {y_norm:.4f} {theta_deg:.2f}")

    pose = detection.get("pose")
    if pose is None:
        return

    print(
        f"TAGPOSE_RAW id={tag_id} x_cam={pose['x_m']:.4f} "
        f"y_cam={pose['y_m']:.4f} z_cam={pose['z_m']:.4f} "
        f"yaw={pose['yaw_deg']:.2f}"
    )

    if not pose["valid"]:
        print(f"TAGPOSE_INVALID id={tag_id} z_cam={pose['z_m']:.4f}")
        return

    print(
        f"TAGPOSE {tag_id} {pose['x_m']:.4f} {pose['y_m']:.4f} "
        f"{pose['yaw_deg']:.2f}"
    )


def draw_debug(frame, detection, x_norm, y_norm, theta_deg):
    h, w = frame.shape[:2]

    corners = np.array(detection["corners"], dtype=np.int32)
    cx = int(detection["center_x"])
    cy = int(detection["center_y"])

    cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
    cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
    cv2.circle(frame, (w // 2, h // 2), 5, (255, 0, 0), -1)

    cv2.line(frame, (w // 2, h // 2), (cx, cy), (255, 255, 0), 2)

    text1 = f"ID:{detection['tag_id']}"
    text2 = f"x:{x_norm:+.3f} y:{y_norm:+.3f} th:{theta_deg:+.2f}"
    pose = detection.get("pose")

    cv2.putText(frame, text1, (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.putText(frame, text2, (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    if pose is not None:
        text3 = (
            f"xm:{pose['x_m']:+.3f} ym:{pose['y_m']:+.3f} "
            f"zm:{pose['z_m']:+.3f} yaw:{pose['yaw_deg']:+.1f}"
        )
        cv2.putText(frame, text3, (20, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)


def main():
    camera = CameraManager()
    detector = AprilTagDetector()

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        time.sleep(2.0)
        print(f"Serial connected: {SERIAL_PORT}")
    except Exception as error:
        print(f"Serial connection failed: {error}")
        return

    if not camera.initialize():
        print("Camera failed to initialize.")
        ser.close()
        return

    if SEND_ALIGN_ON_AT_START:
        ser.write(b"ALIGN ON\n")
        print("Sent: ALIGN ON")
    else:
        ser.write(b"ALIGN OFF\n")
        print("Sent: ALIGN OFF for safe sign checking")

    last_send_time = 0.0
    last_seen_time = time.monotonic()
    lost_sent = False

    try:
        while True:
            frame = camera.get_frame()
            if frame is None:
                print("Frame capture failed.")
                continue

            frame_h, frame_w = frame.shape[:2]
            image_center_x = frame_w / 2.0
            image_center_y = frame_h / 2.0

            detections = detector.detect(frame)
            for detected_tag in detections:
                add_detection_measurements(
                    detected_tag,
                    image_center_x,
                    image_center_y,
                    frame_w,
                    frame_h,
                )

            detection = choose_best_detection(detections)

            now = time.monotonic()
            should_send = (now - last_send_time) >= (1.0 / SEND_RATE_HZ)

            if detection is not None:
                last_seen_time = now
                lost_sent = False

                tag_id = detection["tag_id"]
                x_norm = detection["x_norm"]
                y_norm = detection["y_norm"]
                theta_deg = detection["theta_deg"]

                if should_send:
                    cmd = f"TAG {tag_id} {x_norm:.4f} {y_norm:.4f} {theta_deg:.2f}\n"
                    ser.write(cmd.encode("utf-8"))

                    pose = detection.get("pose")
                    if SEND_TAGPOSE_TO_ESP32 and pose is not None and pose["valid"]:
                        pose_cmd = (
                            f"TAGPOSE {tag_id} {pose['x_m']:.4f} "
                            f"{pose['y_m']:.4f} {pose['yaw_deg']:.2f}\n"
                        )
                        ser.write(pose_cmd.encode("utf-8"))

                    for detected_tag in detections:
                        print_detection_log(detected_tag)

                    last_send_time = now

                draw_debug(frame, detection, x_norm, y_norm, theta_deg)

            else:
                if now - last_seen_time > LOST_DELAY_SEC and not lost_sent:
                    ser.write(b"TAG LOST\n")
                    print("TAG LOST")
                    lost_sent = True

            cv2.imshow("AGV AprilTag Serial Sender", frame)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("a"):
                ser.write(b"ALIGN ON\n")
                print("Sent: ALIGN ON")

            elif key == ord("o"):
                ser.write(b"ALIGN OFF\n")
                print("Sent: ALIGN OFF")

            elif key == ord("s"):
                ser.write(b"S\n")
                print("Sent: STOP")

            elif key == ord("q"):
                ser.write(b"ALIGN OFF\n")
                ser.write(b"S\n")
                print("Quit: ALIGN OFF and STOP sent")
                break

    finally:
        camera.release()
        ser.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
