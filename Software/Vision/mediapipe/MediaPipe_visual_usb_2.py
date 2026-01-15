import cv2
import mediapipe as mp
import numpy as np
import subprocess
import time
import serial
import struct
from collections import deque

# rpicam 관련 프로세스 모두 종료
#pkill -f rpicam

# libcamera 관련 프로세스 모두 종료
#pkill -f libcamera

# 실행 중인 파이썬 코드 모두 종료 (이전에 멈춘 코드가 백그라운드에 있을 수 있음)
#pkill -f python

# ==========================================
# [1] 환경 설정 (Configuration)
# ==========================================
# >> 졸음 판단 임계값
EAR_THRESHOLD = 0.20 
# >> PERCLOS 계산을 위한 히스토리 버퍼 설정
FPS_ESTIMATE = 30             
WINDOW_SECONDS = 5            
MAX_HISTORY = FPS_ESTIMATE * WINDOW_SECONDS 

# >> USB 시리얼 통신 설정
SERIAL_PORT = '/dev/serial0'  
BAUD_RATE = 115200

# >> MediaPipe 눈 좌표 인덱스
LEFT_EYE = [362, 385, 387, 263, 373, 380]
RIGHT_EYE = [33, 160, 158, 133, 153, 144]

# >> 에러 플래그 정의 (Vision_Err_Flag - 4bit)
ERR_OK = 0          # 정상
ERR_CAM_FAIL = 1    # 카메라 고장/연결 끊김
ERR_INIT_FAIL = 2   # 시리얼/초기화 실패

# ==========================================
# [2] 헬퍼 함수 (Helpers)
# ==========================================

def calculate_ear(landmarks, indices, w, h):
    """
    [기능] 눈의 종횡비(EAR) 계산
    """
    coords = []
    for idx in indices:
        lm = landmarks[idx]
        coords.append(np.array([lm.x * w, lm.y * h]))
    
    v1 = np.linalg.norm(coords[1] - coords[5])
    v2 = np.linalg.norm(coords[2] - coords[4])
    h_dist = np.linalg.norm(coords[0] - coords[3])
    
    if h_dist == 0: return 0.0
    return (v1 + v2) / (2.0 * h_dist)

def create_dbc_payload(perclos, eye_state_raw, face_detect_flag, alive_cnt, err_flag):
    """
    [기능] CAN 인터페이스 사양(DBC)에 맞춰 8바이트 배열 생성
    """
    # Byte 0: PERCLOS (0~100)
    byte0 = int(max(0, min(100, perclos)))

    # Byte 1: Eye_State_Raw(Bit 0) | Face_Detect_Flag(Bit 1)
    byte1 = (eye_state_raw & 0x01) | ((face_detect_flag & 0x01) << 1)

    # Byte 2 ~ 6: Reserved (0)
    byte2 = 0
    byte3 = 0
    byte4 = 0
    byte5 = 0
    byte6 = 0

    # Byte 7: Alive_Cnt(Bit 0~3) | Err_Flag(Bit 4~7)
    byte7 = (alive_cnt & 0x0F) | ((err_flag & 0x0F) << 4)

    # 8바이트 패킹
    payload = struct.pack('8B', byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7)
    return payload

def wrap_packet_usb(payload):
    """
    [기능] USB 전송용 래핑 (STX + Payload + Checksum + ETX)
    """
    header = b'\x02'  # STX
    checksum = sum(payload) % 256
    tail = b'\x03'    # ETX
    return header + payload + struct.pack('B', checksum) + tail

# ==========================================
# [3] 메인 루프 (Main Loop)
# ==========================================
def main():
    # 상태 변수 초기화
    vision_err_flag = ERR_OK
    alive_cnt = 0  # 0~15 롤링 카운터
    
    # USB 시리얼 연결
    ser = None
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✅ Serial connected to {SERIAL_PORT}")
    except Exception as e:
        vision_err_flag = ERR_INIT_FAIL
        print(f"⚠️ Serial connection failed: {e}")

    # 카메라 프로세스 실행
    cmd = ['rpicam-vid', '-t', '0', '--width', '640', '--height', '480', 
           '--inline', '--nopreview', '--codec', 'mjpeg', '-o', '-']
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    # MediaPipe 초기화
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(max_num_faces=1, 
                                      refine_landmarks=True,
                                      min_detection_confidence=0.7,
                                      min_tracking_confidence=0.7)

    eye_history = deque(maxlen=MAX_HISTORY)
    bytes_data = b''
    prev_time = 0

    while True:
        # 카메라 영상 데이터 읽기
        bytes_data += process.stdout.read(2048)
        a = bytes_data.find(b'\xff\xd8')
        b = bytes_data.find(b'\xff\xd9')

        if a != -1 and b != -1:
            jpg = bytes_data[a:b+2]
            bytes_data = bytes_data[b+2:]
            
            try:
                image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            except: 
                continue 

            if image is None: continue

            h, w, _ = image.shape
            results = face_mesh.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

            # --- [데이터 처리 시작] ---
            
            face_detect_flag = 0  # 0: None
            eye_state_raw = 0     # 0: Open (미감지일 때 기본값)
            avg_ear = 0.0

            # 1. 얼굴 감지 여부 확인
            if results.multi_face_landmarks:
                face_detect_flag = 1 
                vision_err_flag = ERR_OK 

                for face_landmarks in results.multi_face_landmarks:
                    l_ear = calculate_ear(face_landmarks.landmark, LEFT_EYE, w, h)
                    r_ear = calculate_ear(face_landmarks.landmark, RIGHT_EYE, w, h)
                    avg_ear = (l_ear + r_ear) / 2.0

                    # 시각화 (눈 윤곽선)
                    for eye_indices in [LEFT_EYE, RIGHT_EYE]:
                        pts = np.array([[int(face_landmarks.landmark[i].x * w), 
                                         int(face_landmarks.landmark[i].y * h)] for i in eye_indices])
                        cv2.polylines(image, [pts], True, (0, 255, 255), 1)

                    # 현재 프레임의 눈 상태 판단
                    if avg_ear < EAR_THRESHOLD:
                        eye_state_raw = 1 
                    else:
                        eye_state_raw = 0
                    
                    eye_history.append(eye_state_raw)
            else:
                # 얼굴 미검출
                face_detect_flag = 0
                eye_state_raw = 0   
                # 미검출 시 히스토리 업데이트 안 함

            # 2. PERCLOS 계산 (0~100)
            perclos = 0.0
            if len(eye_history) > 0:
                perclos = (sum(eye_history) / len(eye_history)) * 100

            # 3. Alive Count 증가 (0 ~ 15 반복)
            alive_cnt = (alive_cnt + 1) % 16

            # --- [데이터 송신] ---
            payload_8byte = create_dbc_payload(perclos, eye_state_raw, face_detect_flag, alive_cnt, vision_err_flag)
            final_packet = wrap_packet_usb(payload_8byte)

            if ser is not None:
                try:
                    ser.write(final_packet)
                except: pass 

            # 4. [복구됨] 터미널 로그 출력 (PERCLOS 값 포함)
            payload_hex = ' '.join(f'{b:02X}' for b in payload_8byte)
            print(f"[DATA] {payload_hex} | Alive:{alive_cnt} Detect:{face_detect_flag} PERCLOS:{perclos:.1f}%")

            # --- [화면 시각화] ---
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time

            # FPS 표시
            cv2.putText(image, f"FPS: {int(fps)}", (30, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2)
            
            # [복구됨] PERCLOS 위험도별 색상 적용
            p_color = (0, 255, 0) # 평소 초록
            if perclos > 30: 
                p_color = (0, 0, 255) # 위험 빨강
            elif perclos > 10: 
                p_color = (0, 255, 255) # 주의 노랑

            cv2.putText(image, f"PERCLOS: {perclos:.1f}%", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, p_color, 2)
            
            # 상태 표시
            status_str = "DETECTED" if face_detect_flag else "NONE"
            eye_str = "CLOSED" if eye_state_raw else "OPEN"
            
            color_face = (255, 255, 0) if face_detect_flag else (0, 0, 255)
            cv2.putText(image, f"Face: {status_str}", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_face, 2)
            cv2.putText(image, f"Eye: {eye_str}", (30, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow('Vision Status', image)
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    process.terminate()
    cv2.destroyAllWindows()
    if ser is not None: ser.close()

if __name__ == "__main__":
    main()