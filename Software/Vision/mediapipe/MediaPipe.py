import cv2
import mediapipe as mp
import numpy as np
import subprocess
import time

# --- [1] 환경 설정 ---
# EAR_THRESHOLD: 눈 감음 판단 기준 (실험을 통해 0.20~0.22 사이로 조절)
EAR_THRESHOLD = 0.21 
# SLEEP_FRAMES: 몇 프레임 동안 눈을 감아야 졸음으로 볼 것인가 (30fps 기준 15프레임은 약 0.5초)
SLEEP_FRAMES = 60   

# MediaPipe 고유 인덱스 번호 (눈 윤곽선)
LEFT_EYE = [362, 385, 387, 263, 373, 380]
RIGHT_EYE = [33, 160, 158, 133, 153, 144]

def calculate_ear(landmarks, indices, w, h):
    """
    눈의 종횡비(EAR)를 계산하는 함수
    """
    coords = []
    for idx in indices:
        lm = landmarks[idx]
        # 비율(0~1)로 들어오는 좌표를 실제 이미지 픽셀 좌표(w, h 곱하기)로 변환
        coords.append(np.array([lm.x * w, lm.y * h]))
    
    # 눈꺼풀 위아래 거리 (수직) - 두 쌍의 평균
    v1 = np.linalg.norm(coords[1] - coords[5])
    v2 = np.linalg.norm(coords[2] - coords[4])
    # 눈 양 끝 거리 (수평)
    h_dist = np.linalg.norm(coords[0] - coords[3])
    
    # 수직 거리를 수평 거리로 나눈 비율 (눈이 작아지면 이 값도 작아짐)
    return (v1 + v2) / (2.0 * h_dist)

def main():
    # --- [2] 라즈베리파이 5 전용 카메라 파이프라인 ---
    # rpicam-vid 명령어를 통해 영상을 MJPEG 형식으로 가져와서 파이썬으로 쏩니다.
    cmd = ['rpicam-vid', '-t', '0', '--width', '640', '--height', '480', 
           '--inline', '--nopreview', '--codec', 'mjpeg', '-o', '-']
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    # --- [3] MediaPipe 얼굴망(Face Mesh) 초기화 ---
    mp_face_mesh = mp.solutions.face_mesh
    # refine_landmarks=True는 눈동자와 눈꺼풀 정밀 인덱스를 활성화합니다.
    face_mesh = mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True)

    sleep_counter = 0
    bytes_data = b''

    prev_time = 0

    while True:
        # 데이터 스트림에서 한 프레임(JPEG) 분량을 찾아내는 과정
        bytes_data += process.stdout.read(2048)
        a = bytes_data.find(b'\xff\xd8') # JPEG 시작 지점
        b = bytes_data.find(b'\xff\xd9') # JPEG 끝 지점

        if a != -1 and b != -1:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            # 바이너리 데이터를 이미지 객체로 변환
            image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            if image is None: continue

            h, w, _ = image.shape
            # MediaPipe 처리를 위해 RGB로 변환 (OpenCV는 기본이 BGR)
            results = face_mesh.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    # EAR 계산
                    l_ear = calculate_ear(face_landmarks.landmark, LEFT_EYE, w, h)
                    r_ear = calculate_ear(face_landmarks.landmark, RIGHT_EYE, w, h)
                    avg_ear = (l_ear + r_ear) / 2.0

                    # 눈 시각화 (노란색 선)
                    for eye_indices in [LEFT_EYE, RIGHT_EYE]:
                        pts = np.array([[int(face_landmarks.landmark[i].x * w), 
                                         int(face_landmarks.landmark[i].y * h)] for i in eye_indices])
                        cv2.polylines(image, [pts], True, (0, 255, 255), 1)

                    # --- [4] 화면 표시 부분 (위치 수정) ---
                    # EAR 수치 표시
                    ear_text = f"EAR: {avg_ear:.2f}"
                    cv2.putText(image, ear_text, (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                    # 졸음 판단 및 "SLEEPING!" 문구 표시 (EAR 수치 옆에 배치)
                    if avg_ear < EAR_THRESHOLD:
                        sleep_counter += 1
                        if sleep_counter > SLEEP_FRAMES:
                            # (200, 30) 위치에 EAR과 동일한 크기(0.7)로 빨간색 표시
                            cv2.putText(image, "SLEEPING!", (200, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    else:
                        sleep_counter = 0
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time)
            prev_time = curr_time
            cv2.putText(image, f"FPS: {int(fps)}", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

            cv2.imshow('Sleep Detection', image)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    process.terminate()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()