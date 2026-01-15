import cv2
import mediapipe as mp
import numpy as np
import subprocess
import os

# --- 설정 ---
EAR_THRESHOLD = 0.21
SLEEP_FRAMES = 20
LEFT_EYE = [362, 385, 387, 263, 373, 380]
RIGHT_EYE = [33, 160, 158, 133, 153, 144]

def calculate_ear(landmarks, indices, w, h):
    coords = []
    for idx in indices:
        lm = landmarks[idx]
        coords.append(np.array([lm.x * w, lm.y * h]))
    v1 = np.linalg.norm(coords[1] - coords[5])
    v2 = np.linalg.norm(coords[2] - coords[4])
    h_dist = np.linalg.norm(coords[0] - coords[3])
    return (v1 + v2) / (2.0 * h_dist)

def main():
    # 1. 카메라 명령어 설정 (raw 데이터를 stdout으로 출력)
    # libcamera-vid 대신 더 가벼운 rpicam-vid 사용
    cmd = [
        'rpicam-vid',
        '-t', '0',
        '--width', '640',
        '--height', '480',
        '--inline',
        '--nopreview',
        '--codec', 'mjpeg',
        '-o', '-'  # 출력 위치를 stdout(표준 출력)으로 설정
    ]
    
    # 프로세스 시작
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    # 2. MediaPipe 설정
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(
        max_num_faces=1,
        refine_landmarks=True,
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5
    )

    sleep_counter = 0
    bytes_data = b''

    print("🚀 라즈베리파이 5 카메라 파이프라인 시작...")

    try:
        while True:
            # MJPEG 스트림에서 한 프레임씩 읽기 (FFD8 ~ FFD9 찾기)
            bytes_data += process.stdout.read(1024)
            a = bytes_data.find(b'\xff\xd8')
            b = bytes_data.find(b'\xff\xd9')

            if a != -1 and b != -1:
                jpg = bytes_data[a:b+2]
                bytes_data = bytes_data[b+2:]
                
                # 이미지 디코딩
                image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if image is None: continue

                h, w, _ = image.shape
                image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                results = face_mesh.process(image_rgb)

                if results.multi_face_landmarks:
                    for face_landmarks in results.multi_face_landmarks:
                        left_ear = calculate_ear(face_landmarks.landmark, LEFT_EYE, w, h)
                        right_ear = calculate_ear(face_landmarks.landmark, RIGHT_EYE, w, h)
                        avg_ear = (left_ear + right_ear) / 2.0

                        if avg_ear < EAR_THRESHOLD:
                            sleep_counter += 1
                        else:
                            sleep_counter = 0

                        if sleep_counter > SLEEP_FRAMES:
                            cv2.putText(image, "SLEEPING ALERT!", (50, 100),
                                        cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)

                        cv2.putText(image, f'EAR: {avg_ear:.2f}', (30, 30),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                cv2.imshow('Pi 5 Sleep Monitor (Pipe)', image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

    except Exception as e:
        print(f"Error: {e}")
    finally:
        process.terminate()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()