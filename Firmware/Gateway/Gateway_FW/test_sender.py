import serial
import time
import struct

# 시리얼 포트 설정 (환경에 맞게 수정)
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

print("UART Sender Started (V0.8 Protocol)...")
print("Structure Size: 10 Bytes") 

cnt = 0

try:
    while True:
        # === 1. 데이터 생성 ===
        header = 0xFF          # 고정 헤더
        perclos = (cnt * 5) % 100
        
        # [비트필드 1 조립] Byte 2
        # eye_state (1bit) + face_flag (1bit) + reserved (6bit)
        eye_state = cnt % 2    # 0 or 1
        face_flag = 1          # 1: Detected
        
        # 파이썬은 비트필드가 없으므로 시프트 연산으로 만듭니다.
        # 예: face_flag(1)을 왼쪽으로 1칸 밀고, eye_state(0)와 합침
        # 00000010 | 00000000 = 00000010 (0x02)
        byte_2_bitfield = (eye_state & 0x01) | ((face_flag & 0x01) << 1)
        
        # [예비 영역] Byte 3~7 (5바이트)
        reserved_arr = [0, 0, 0, 0, 0]
        
        # [비트필드 2 조립] Byte 8
        # alive_cnt (4bit) + err_flag (4bit)
        alive_cnt = cnt % 16   # 0 ~ 15
        err_flag = 0           # 에러 없음
        
        # err_flag를 왼쪽으로 4칸 밀어서 상위 비트로 보냄
        byte_8_bitfield = (alive_cnt & 0x0F) | ((err_flag & 0x0F) << 4)
        
        # === 2. 체크섬 계산 (XOR) ===
        # 전송할 데이터 리스트 (헤더부터 에러플래그까지)
        data_payload = [header, perclos, byte_2_bitfield] + reserved_arr + [byte_8_bitfield]
        
        checksum = 0
        for byte_val in data_payload:
            checksum ^= byte_val # 모든 바이트를 XOR 연산
            
        # === 3. 패킹 및 전송 (총 10바이트) ===
        # Format: < (리틀엔디안)
        # B (Header)
        # B (Perclos)
        # B (Eye/Face Bitfield)
        # 5B (Reserved Array - 5개)
        # B (Alive/Err Bitfield)
        # B (Checksum)
        packet = struct.pack('<BBB5BBB', 
                             header, 
                             perclos, 
                             byte_2_bitfield, 
                             *reserved_arr, # 리스트 언패킹
                             byte_8_bitfield, 
                             checksum)
        
        ser.write(packet)
        
        # 디버깅 출력
        print(f"Sent: PERCLOS={perclos}, Eye={eye_state}, Alive={alive_cnt}, CKSM={checksum:02X}")
        
        cnt += 1
        time.sleep(0.5)

except KeyboardInterrupt:
    ser.close()
    print("\nStopped.")