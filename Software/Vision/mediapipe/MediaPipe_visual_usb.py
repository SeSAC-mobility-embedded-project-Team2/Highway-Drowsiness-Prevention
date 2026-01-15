import cv2                  # OpenCV 라이브러리: 이미지 처리 및 시각화 (선 그리기, 글씨 쓰기 등)
import mediapipe as mp      # 구글 MediaPipe: 얼굴 랜드마크(눈, 코, 입 좌표) 추출 AI 모델
import numpy as np          # NumPy: 좌표 계산 및 행렬 연산 (유클리드 거리 계산 등)
import subprocess           # Subprocess: 라즈베리파이 카메라 명령어(rpicam-vid)를 파이썬에서 실행하기 위함
import time                 # Time: FPS(초당 프레임 수) 계산 및 시간 측정
import serial               # PySerial: STM32와의 USB 시리얼 통신 라이브러리
import struct               # Struct: 데이터를 C언어 구조체(Binary) 형태로 변환 (패킷 압축)
from collections import deque # Deque: 데이터를 밀어내기 식으로 저장하는 큐 (최근 3초 데이터 유지용)

# ==========================================
# [1] 환경 설정 및 상수 정의
# ==========================================

# >> 졸음 판단 임계값 설정
# EAR(Eye Aspect Ratio): 눈이 얼마나 떠져있는지 나타내는 비율 (0.0 ~ 1.0)
# 보통 눈을 뜨면 0.3 이상, 감으면 0.2 이하로 떨어짐. 실험적으로 0.21로 설정함.
EAR_THRESHOLD = 0.20   

# >> PERCLOS(Percentage of Eyelid Closure) 계산 설정
# PERCLOS는 "최근 일정 시간 동안 눈을 감은 비율"을 의미함.
FPS_ESTIMATE = 30             # 라즈베리파이 5의 예상 프레임 속도 (약 30FPS)
WINDOW_SECONDS = 3            # PERCLOS를 계산할 시간 범위 (최근 3초)
# 큐(Queue)의 최대 크기: 3초 * 30FPS = 약 90개의 프레임 데이터를 저장
MAX_HISTORY = FPS_ESTIMATE * WINDOW_SECONDS 

# >> USB 시리얼 통신 설정 (STM32 연결용)
# 라즈베리파이와 STM32를 USB 케이블로 연결하면 '/dev/ttyACM0'로 잡힘
SERIAL_PORT = '/dev/ttyACM0'  
BAUD_RATE = 115200            # 통신 속도 (STM32 설정과 일치해야 함)

# >> MediaPipe 얼굴 랜드마크 인덱스 (468개 점 중 눈 주변 점 번호)
# 왼쪽 눈 점 6개, 오른쪽 눈 점 6개 좌표 인덱스
LEFT_EYE = [362, 385, 387, 263, 373, 380]
RIGHT_EYE = [33, 160, 158, 133, 153, 144]

# ==========================================
# [2] 헬퍼 함수 정의
# ==========================================

def calculate_ear(landmarks, indices, w, h):
    """
    [기능] 눈의 종횡비(EAR)를 계산하는 함수
    EAR = (수직 거리 1 + 수직 거리 2) / (2 * 수평 거리)
    눈을 뜨면 값이 크고, 감으면 값이 작아짐.
    """
    coords = []
    for idx in indices:
        lm = landmarks[idx]
        # MediaPipe는 좌표를 0~1 비율로 반환하므로, 실제 픽셀 좌표(w, h)로 변환 필요
        coords.append(np.array([lm.x * w, lm.y * h]))

    # 눈꺼풀 위아래 거리 (수직) - 두 쌍의 수직 거리를 구해 평균 냄
    v1 = np.linalg.norm(coords[1] - coords[5])
    v2 = np.linalg.norm(coords[2] - coords[4])

    # 눈 양 끝 거리 (수평)
    h_dist = np.linalg.norm(coords[0] - coords[3])

    # 0으로 나누기 방지 및 EAR 반환
    if h_dist == 0: return 0.0
    return (v1 + v2) / (2.0 * h_dist)

def create_binary_packet(detect, perclos_val):
    """
    [기능] STM32로 보낼 데이터를 '8바이트 바이너리 패킷'으로 변환
    [구조] STX(1) | DETECT(1) | PERCLOS(4) | CHECKSUM(1) | ETX(1)
    
    - 텍스트("25.5")로 보내는 것보다 바이트로 보내는 것이 훨씬 빠르고 정확함.
    - C언어 구조체와 1:1 매핑됨.
    """
    # 1. Header (STX): 패킷의 시작을 알리는 고정 값 (0x02)
    header = b'\x02'
    
    # 2. Payload: 실제 데이터 (총 5바이트)
    # '<Bf' 포맷 의미:
    # < : 리틀 엔디안 (STM32와 호환성 위해)
    # B : unsigned char (1바이트) -> detect (0 또는 1)
    # f : float (4바이트) -> perclos_val (실수형 데이터)
    payload = struct.pack('<Bf', detect, perclos_val)
    
    # 3. Checksum: 데이터 무결성 검사 (Payload 바이트들의 합계)
    # 중간에 노이즈로 데이터가 깨졌는지 STM32가 확인할 수 있게 함
    checksum_val = sum(payload) % 256
    checksum = struct.pack('B', checksum_val)
    
    # 4. Tail (ETX): 패킷의 끝을 알리는 고정 값 (0x03)
    tail = b'\x03'
    
    # 최종적으로 바이트들을 이어 붙여 반환
    return header + payload + checksum + tail

# ==========================================
# [3] 메인 실행 함수
# ==========================================
def main():
    # --- 시리얼 포트 연결 시도 ---
    ser = None
    try:
        # 타임아웃 0.1초: 연결 안 되면 프로그램 멈추지 않고 바로 넘어가기 위함
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
        print(f"✅ Serial connected to {SERIAL_PORT}")
    except Exception as e:
        print(f"⚠️ Serial connection failed: {e}")
        print("➡️  STM32 연결 없이 시뮬레이션 모드로 동작합니다. (터미널 출력 확인)")

    # --- 카메라 프로세스 시작 (rpicam-vid) ---
    # 라즈베리파이 5 전용 명령어인 rpicam-vid를 사용하여 MJPEG 스트림을 생성
    # -t 0: 무한 촬영, --width/height: 해상도 640x480 (속도 최적화)
    # -o -: 영상을 파일이 아닌 표준 출력(stdout)으로 내보냄 -> 파이썬이 읽음
    cmd = ['rpicam-vid', '-t', '0', '--width', '640', '--height', '480', 
           '--inline', '--nopreview', '--codec', 'mjpeg', '-o', '-']
    
    # subprocess를 통해 백그라운드에서 카메라 실행
    process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)

    # --- MediaPipe 초기화 ---
    # max_num_faces=1: 운전자 1명만 감지
    # refine_landmarks=True: 눈동자 등 정밀 랜드마크 활성화
    mp_face_mesh = mp.solutions.face_mesh
    face_mesh = mp_face_mesh.FaceMesh(max_num_faces=1, refine_landmarks=True)

    # PERCLOS 계산용 큐 (최근 90프레임 눈 상태 저장)
    eye_history = deque(maxlen=MAX_HISTORY)
    
    # MJPEG 스트림 버퍼 및 시간 변수
    bytes_data = b''
    prev_time = 0

    # ======================================
    # 무한 루프: 실시간 영상 처리 시작
    # ======================================
    while True:
        # 1. 카메라 데이터 읽기 (스트림에서 JPEG 이미지 한 장 추출)
        bytes_data += process.stdout.read(2048) # 데이터를 덩어리로 읽어옴
        a = bytes_data.find(b'\xff\xd8') # JPEG 시작 바이너리 (SOI)
        b = bytes_data.find(b'\xff\xd9') # JPEG 끝 바이너리 (EOI)

        # 온전한 JPEG 이미지를 찾았을 때만 처리
        if a != -1 and b != -1:
            jpg = bytes_data[a:b+2]      # 이미지 데이터 잘라내기
            bytes_data = bytes_data[b+2:] # 처리한 데이터 버림 (다음 프레임 위해)
            
            try:
                # 바이너리 데이터를 OpenCV 이미지 객체로 디코딩
                image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            except: continue
            if image is None: continue

            h, w, _ = image.shape # 이미지 크기 (높이, 너비)

            # 2. MediaPipe 얼굴 감지 수행
            # OpenCV는 BGR 색상, MediaPipe는 RGB 색상을 쓰므로 변환 필요
            results = face_mesh.process(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

            # --- 변수 초기화 ---
            face_detected = 0   # 0: 미감지, 1: 감지
            avg_ear = 0.0
            
            # 3. 얼굴 감지 성공 시 로직
            if results.multi_face_landmarks:
                face_detected = 1 # [감지 상태: ON]
                
                for face_landmarks in results.multi_face_landmarks:
                    # 왼쪽 눈, 오른쪽 눈 EAR 계산
                    l_ear = calculate_ear(face_landmarks.landmark, LEFT_EYE, w, h)
                    r_ear = calculate_ear(face_landmarks.landmark, RIGHT_EYE, w, h)
                    
                    # 두 눈의 평균 EAR 사용
                    avg_ear = (l_ear + r_ear) / 2.0

                    # 화면에 눈 윤곽선 그리기 (노란색)
                    for eye_indices in [LEFT_EYE, RIGHT_EYE]:
                        pts = np.array([[int(face_landmarks.landmark[i].x * w), 
                                         int(face_landmarks.landmark[i].y * h)] for i in eye_indices])
                        cv2.polylines(image, [pts], True, (0, 255, 255), 1)

                    # PERCLOS 큐에 현재 상태 저장
                    # 임계값(0.21)보다 작으면 감음(1), 크면 뜸(0)
                    eye_history.append(1 if avg_ear < EAR_THRESHOLD else 0)
            else:
                # 얼굴 감지 실패 시 (고개 돌림, 이탈 등)
                face_detected = 0 # [감지 상태: OFF]
                # *중요* 얼굴을 놓쳤을 때는 PERCLOS 큐에 데이터를 넣지 않음 (이전 상태 유지)
                # 만약 놓쳤을 때 '눈 뜸(0)'으로 처리하면 졸음 수치가 급격히 떨어지는 왜곡 발생 가능

            # 4. PERCLOS 계산 (최근 3초 데이터 평균)
            perclos = 0.0
            if len(eye_history) > 0:
                # (눈 감은 횟수 / 전체 데이터 수) * 100 = 백분율
                perclos = (sum(eye_history) / len(eye_history)) * 100

            # 5. 데이터 송신 (STM32로 전송)
            # 8바이트 바이너리 패킷 생성
            packet = create_binary_packet(face_detected, perclos)

            # (1) 터미널 출력: 사용자가 눈으로 데이터 확인용
            # 패킷 내용을 16진수(HEX)로 변환하여 출력 (예: 02 01 00 ... 03)
            hex_output = ' '.join(f'{b:02X}' for b in packet)
            print(f"[SENT HEX] {hex_output} | Detect: {face_detected}, Perclos: {perclos:.1f}%")

            # (2) USB 시리얼 전송: 실제 데이터 쏘기
            if ser is not None:
                try:
                    ser.write(packet) # 바이너리 전송
                except: pass

            # 6. 화면 시각화 (GUI)
            # FPS(초당 프레임) 계산
            curr_time = time.time()
            fps = 1 / (curr_time - prev_time) if (curr_time - prev_time) > 0 else 0
            prev_time = curr_time
            cv2.putText(image, f"FPS: {int(fps)}", (30, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (150, 150, 150), 2)

            # PERCLOS 상태에 따라 글씨 색상 변경 (초록 -> 노랑 -> 빨강)
            color_p = (0, 255, 0) # 정상 (초록)
            if perclos > 30: color_p = (0, 0, 255)       # 위험 (빨강)
            elif perclos > 10: color_p = (0, 255, 255)   # 주의 (노랑)

            # 텍스트 정보 표시
            cv2.putText(image, f"EAR: {avg_ear:.2f}", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(image, f"PERCLOS(3s): {perclos:.1f}%", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color_p, 2)
            
            # 얼굴 감지 여부 표시
            if face_detected:
                cv2.putText(image, "Face: DETECTED", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            else:
                cv2.putText(image, "Face: LOST", (30, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            # 윈도우 창에 이미지 출력
            cv2.imshow('Sleep Detection', image)
            
            # 'q' 키를 누르면 루프 종료
            if cv2.waitKey(1) & 0xFF == ord('q'): break

    # 프로그램 종료 시 리소스 정리
    process.terminate()       # 카메라 프로세스 종료
    cv2.destroyAllWindows()   # 윈도우 창 닫기
    if ser is not None: ser.close() # 시리얼 포트 닫기

if __name__ == "__main__":
    main()