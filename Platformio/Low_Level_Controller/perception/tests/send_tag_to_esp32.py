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

def calculate_tag_area(corners):
    pts = np.array(corners, dtype=np.float32)
    return abs(cv2.contourArea(pts))


def choose_best_detection(detections):
    if not detections:
        return None

    # Pick largest visible tag area. Good for floor tag when only one should matter.
    return max(detections, key=lambda d: calculate_tag_area(d["corners"]))


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

    cv2.putText(frame, text1, (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    cv2.putText(frame, text2, (20, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)


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
            detection = choose_best_detection(detections)

            now = time.monotonic()
            should_send = (now - last_send_time) >= (1.0 / SEND_RATE_HZ)

            if detection is not None:
                last_seen_time = now
                lost_sent = False

                tag_id = detection["tag_id"]
                tag_center_x = detection["center_x"]
                tag_center_y = detection["center_y"]

                x_error_px = tag_center_x - image_center_x
                y_error_px = tag_center_y - image_center_y

                x_norm = x_error_px / image_center_x
                y_norm = y_error_px / image_center_y

                theta_deg = detection.get("orientation_deg")
                if theta_deg is None:
                    theta_deg = 0.0

                if should_send:
                    cmd = f"TAG {tag_id} {x_norm:.4f} {y_norm:.4f} {theta_deg:.2f}\n"
                    ser.write(cmd.encode("utf-8"))
                    print(cmd.strip())
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
