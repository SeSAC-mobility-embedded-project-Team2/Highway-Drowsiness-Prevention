import serial
import time
import struct

# 사용자가 설정한 115200 속도 적용!
ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=1)

print("=== Scenario Tester Started ===")
print("Flow: Normal (5s) -> Drowsy (5s) -> Repeat")

cnt = 0

try:
    while True:
        # === 상황 연출 ===
        scenario_timer = cnt % 10 # 0~9 반복
        
        if scenario_timer < 5:
            # [0~4초] 정상 상태 (Normal)
            perclos = 10
            eye_state = 0 # Open
            print(f"[{cnt}] Sending: 🟢 NORMAL (Eye Open)")
        else:
            # [5~9초] 졸음 상태 (Drowsy)
            perclos = 90  # 임계값 80 초과
            eye_state = 1 # Closed
            print(f"[{cnt}] Sending: 🟡 DROWSY (Eye Closed)")

        # === 패킷 조립 (기존과 동일) ===
        header = 0xFF
        face_flag = 1
        
        # 비트필드 조립
        byte_2_bitfield = (eye_state & 0x01) | ((face_flag & 0x01) << 1)
        
        reserved_arr = [0, 0, 0, 0, 0]
        alive_cnt = cnt % 16
        err_flag = 0
        byte_8_bitfield = (alive_cnt & 0x0F) | ((err_flag & 0x0F) << 4)
        
        # 체크섬 계산
        data_payload = [header, perclos, byte_2_bitfield] + reserved_arr + [byte_8_bitfield]
        checksum = 0
        for byte_val in data_payload:
            checksum ^= byte_val

        # 전송
        packet = struct.pack('<BBB5BBB', 
                             header, perclos, byte_2_bitfield, 
                             *reserved_arr, byte_8_bitfield, checksum)
        
        ser.write(packet)
        cnt += 1
        time.sleep(0.5) # 1초 단위로 상태 변경

except KeyboardInterrupt:
    ser.close()
    print("\nTest Stopped.")