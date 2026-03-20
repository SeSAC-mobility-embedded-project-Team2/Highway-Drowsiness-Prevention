import serial
import time
import struct

# ==========================================
# [설정] 라즈베리파이 UART 포트
# RPi 5/4의 기본 GPIO 시리얼은 보통 '/dev/serial0' 입니다.
# ==========================================
PORT = '/dev/ttyAMA0' 
BAUDRATE = 115200

try:
    ser = serial.Serial(PORT, BAUDRATE, timeout=1)
    print(f"✅ Vision Sender Started on {PORT} at {BAUDRATE}bps")
except Exception as e:
    print(f"❌ Error opening serial port: {e}")
    exit()

def send_vision_data():
    cnt = 0
    
    try:
        while True:
            # ---------------------------------------------------
            # 1. 가짜 데이터 생성 (시뮬레이션)
            # ---------------------------------------------------
            
            # [Byte 0] PERCLOS (0 ~ 100 반복)
            perclos = cnt % 101 
            
            # [Byte 1] Bitfield (Eye, Face)
            # 짝수초엔 눈 뜸(0), 홀수초엔 눈 감음(1)
            eye_state = 1 if (cnt // 10) % 2 == 0 else 0 
            face_detect = 1 # 항상 얼굴 인식됨
            
            # 비트 연산: (Face << 1) | Eye
            # 예: Face(1), Eye(0) -> 00000010 (0x02)
            byte_1_flags = (eye_state & 0x01) | ((face_detect & 0x01) << 1)

            # [Byte 7] Alive Count & Err Flag
            alive_cnt = cnt % 16 # 0~15 롤링 카운터
            err_flag = 0         # 0: OK
            
            # 상위 4비트: Error, 하위 4비트: Alive
            byte_7_status = (alive_cnt & 0x0F) | ((err_flag & 0x0F) << 4)


            # ---------------------------------------------------
            # 2. 패킷 조립 (ICD V0.1.1 - 8 Bytes)
            # ---------------------------------------------------
            # B: unsigned char (1 byte)
            # 총 8개: PERCLOS, Flags, Reserved(5개), Status
            packet = struct.pack('8B', 
                                 perclos,       # Byte 0
                                 byte_1_flags,  # Byte 1
                                 0, 0, 0, 0, 0, # Byte 2~6 (Reserved)
                                 byte_7_status  # Byte 7
                                 )

            # ---------------------------------------------------
            # 3. 전송
            # ---------------------------------------------------
            ser.write(packet)
            
            # 디버깅 출력
            print(f"[TX] PERCLOS:{perclos:3d}% | Eye:{eye_state} | Face:{face_detect} | Alive:{alive_cnt}")

            cnt += 1
            time.sleep(0.1) # 100ms 주기 (10Hz)

    except KeyboardInterrupt:
        print("\n🛑 Stopped by User")
        ser.close()

if __name__ == "__main__":
    send_vision_data()