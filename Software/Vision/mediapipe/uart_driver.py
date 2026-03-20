import serial
import struct
import threading
import time

# 1. 시리얼 포트 설정 (STM32와 연결된 포트)
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1.0)

# 2. 패킷 구조 정의 (32바이트)
# H(2) + 5B + 2f + 2B + 3f + 2B + B(1)
# 총합 = 2 + 5 + (4*2) + 2 + (4*3) + 2 + 1 = 32 bytes
PACKET_FORMAT = '<H 5B 2f 2B 3f 2B B'
PACKET_SIZE = struct.calcsize(PACKET_FORMAT) # 32

def read_uart_thread():
    print(f"Listening... Packet Size: {PACKET_SIZE} bytes")
    
    while True:
        try:
            # 버퍼에 데이터가 충분한지 확인
            if ser.in_waiting >= PACKET_SIZE:
                
                # 1. 헤더 찾기 (Sync) - 데이터 밀림 방지
                # 0xFC(252), 0xFD(253)를 찾을 때까지 1바이트씩 읽음
                while True:
                    if ser.in_waiting < 2: break
                    header_check = ser.read(2)
                    if header_check == b'\xfc\xfd': # 리틀엔디안에서는 0xFCFD가 \xfd\xfc일수도 있으나 바이트단위는 순서대로 옴
                        # 헤더 찾음! 나머지 30바이트 읽기
                        raw_data = ser.read(PACKET_SIZE - 2)
                        full_packet = header_check + raw_data
                        
                        # 2. 파싱 (Unpacking)
                        data = struct.unpack(PACKET_FORMAT, full_packet)
                        
                        # 3. 데이터 분해 및 사용
                        header = data[0]
                        # Vision (Index 1~5)
                        perclos = data[1]
                        face_det = data[3]
                        
                        # Chassis (Index 6~9)
                        steer_std = data[6]
                        steer_angle = data[7]
                        
                        # Body (Index 10~14)
                        hands_off = data[11]
                        
                        # Risk (Index 15)
                        risk_level = data[15]

                        print(f"[RX] Risk:{risk_level} | Steer:{steer_angle:.1f} | PERCLOS:{perclos}")
                        break
                    else:
                        # 헤더가 아니면 1바이트 뒤로 이동 (슬라이딩 윈도우)
                        ser.read(1) # 버퍼 비우기용은 아님, seek 기능이 없으므로 로직상 구현 필요
                        # 실제로는 read(1)을 루프 돌며 0xFD, 0xFC 순서를 찾는게 더 정석입니다.
                        pass
                        
        except Exception as e:
            print(f"Error: {e}")
            time.sleep(0.1)

# 스레드 시작
t = threading.Thread(target=read_uart_thread)
t.daemon = True
t.start()

# 메인 루프 (여기서 GUI를 그리면 됨)
while True:
    time.sleep(1)