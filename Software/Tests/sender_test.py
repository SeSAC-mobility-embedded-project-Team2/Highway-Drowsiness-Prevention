import serial
import time
import struct

# 루프백 테스트에서 성공했던 설정 그대로!
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

print("Scenario Tester Started...")
print("0~5s: Normal / 5~10s: Drowsy / 10s~: Repeat")

cnt = 0

try:
    while True:
        # === 시나리오 데이터 생성 ===
        cycle = cnt % 20 # 0 ~ 19 (총 20단계, 약 10초 주기)
        
        if cycle < 10:
            # [Normal] 눈 뜨고 있음, PERCLOS 낮음
            perclos = 10
            eye_state = 0 # OPEN
            print(f"[{cnt}] Sending: NORMAL")
        else:
            # [Drowsy] 눈 감음, PERCLOS 높음
            perclos = 90  # 80초과 -> 경고 조건
            eye_state = 1 # CLOSED
            print(f"[{cnt}] Sending: DROWSY (Warning!)")

        # === 패킷 조립 (이전과 동일) ===
        header = 0xFF
        face_flag = 1
        byte_2_bitfield = (eye_state & 0x01) | ((face_flag & 0x01) << 1)
        reserved_arr = [0, 0, 0, 0, 0]
        alive_cnt = cnt % 16
        err_flag = 0
        byte_8_bitfield = (alive_cnt & 0x0F) | ((err_flag & 0x0F) << 4)
        
        data_payload = [header, perclos, byte_2_bitfield] + reserved_arr + [byte_8_bitfield]
        checksum = 0
        for byte_val in data_payload:
            checksum ^= byte_val

        packet = struct.pack('<BBB5BBB', 
                             header, 
                             perclos,
                             byte_2_bitfield, 
                             *reserved_arr, 
                             byte_8_bitfield, 
                             checksum)
        
        ser.write(packet)
        cnt += 1
        time.sleep(0.5) # STM32 속도와 동기화

except KeyboardInterrupt:
    ser.close()