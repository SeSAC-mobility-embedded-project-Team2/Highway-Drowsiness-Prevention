import socket
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

# ==========================================
# [1] 설정
# ==========================================
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
MAX_LEN = 100  # 최근 100개 데이터 표시

# 패킷 포맷 (32 bytes)
# <H(Header) 5B(Vision) 2f(Chassis_F) 2B(Chassis_I) 3f(Body_F) 2B(Body_I) B(Risk)
FMT = '<H 5B 2f 2B 3f 2B B'

# ------------------------------------------------
# 데이터 저장소 (Deque) 초기화 - 모든 필드 생성
# ------------------------------------------------
# [Vision]
data_perclos = deque([0]*MAX_LEN, maxlen=MAX_LEN)
data_eye     = deque([0]*MAX_LEN, maxlen=MAX_LEN) # Bool
data_face    = deque([0]*MAX_LEN, maxlen=MAX_LEN) # Bool
data_v_alive = deque([0]*MAX_LEN, maxlen=MAX_LEN)
data_v_err   = deque([0]*MAX_LEN, maxlen=MAX_LEN)

# [Chassis]
data_st_std   = deque([0]*MAX_LEN, maxlen=MAX_LEN)
data_st_angle = deque([0]*MAX_LEN, maxlen=MAX_LEN)
data_c_alive  = deque([0]*MAX_LEN, maxlen=MAX_LEN)
data_c_err    = deque([0]*MAX_LEN, maxlen=MAX_LEN)

# [Body]
data_head     = deque([0]*MAX_LEN, maxlen=MAX_LEN)
data_hands    = deque([0]*MAX_LEN, maxlen=MAX_LEN)
data_noop     = deque([0]*MAX_LEN, maxlen=MAX_LEN)
data_b_alive  = deque([0]*MAX_LEN, maxlen=MAX_LEN)
data_b_err    = deque([0]*MAX_LEN, maxlen=MAX_LEN)

# [Result]
data_risk     = deque([0]*MAX_LEN, maxlen=MAX_LEN)

# ==========================================
# [2] UDP 수신 스레드
# ==========================================
def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"대시보드(전체 데이터) 대기 중... ({UDP_PORT})")
    
    while True:
        data, addr = sock.recvfrom(1024)
        if len(data) == 32:
            try:
                p = struct.unpack(FMT, data)
                # p[0]은 Header이므로 생략
                
                # Vision (Index 1~5)
                data_perclos.append(p[1])
                data_eye.append(p[2])
                data_face.append(p[3])
                data_v_alive.append(p[4])
                data_v_err.append(p[5])
                
                # Chassis (Index 6~9)
                data_st_std.append(p[6])
                data_st_angle.append(p[7])
                data_c_alive.append(p[8])
                data_c_err.append(p[9])
                
                # Body (Index 10~14)
                data_head.append(p[10])
                data_hands.append(p[11])
                data_noop.append(p[12])
                data_b_alive.append(p[13])
                data_b_err.append(p[14])
                
                # Result (Index 15)
                data_risk.append(p[15])
                
            except struct.error:
                pass

t = threading.Thread(target=udp_listener, daemon=True)
t.start()

# ==========================================
# [3] 그래프 설정 (Matplotlib 4x4 Grid)
# ==========================================
# 화면 크기를 키움 (16 x 10 inch)
fig, axes = plt.subplots(4, 4, figsize=(16, 10))
fig.suptitle('Full System Monitor (All Data)', fontsize=16)

# axes를 1차원 리스트로 펴서 관리하기 쉽게 만듦
axs = axes.flatten() 

# 라인 객체들을 저장할 리스트
lines = []

# 각 그래프의 설정값 (제목, Y축 범위, 색상, 데이터큐)
# 순서는 위에서 unpack 한 순서가 아니라, 보기 좋은 순서로 배치
plots_config = [
    # Row 1: Main Status
    {"title": "Risk Level (0-100)",  "ylim": (0, 100),   "color": "r", "data": data_risk},
    {"title": "PERCLOS (%)",         "ylim": (0, 100),   "color": "m", "data": data_perclos},
    {"title": "Steering Angle",      "ylim": (-600, 600),"color": "b", "data": data_st_angle},
    {"title": "Head Delta (cm)",     "ylim": (-50, 50),  "color": "g", "data": data_head},

    # Row 2: Secondary Metrics
    {"title": "Hands Off (sec)",     "ylim": (0, 30),    "color": "g", "data": data_hands},
    {"title": "No Operation (sec)",  "ylim": (0, 30),    "color": "g", "data": data_noop},
    {"title": "Steer Std Dev",       "ylim": (0, 10),    "color": "b", "data": data_st_std},
    {"title": "Face Detect (0/1)",   "ylim": (-0.2, 1.2),"color": "m", "data": data_face},

    # Row 3: Flags & Boolean
    {"title": "Eye Closed (0/1)",    "ylim": (-0.2, 1.2),"color": "m", "data": data_eye},
    {"title": "Vision Err Flag",     "ylim": (-1, 5),    "color": "k", "data": data_v_err},
    {"title": "Chassis Err Flag",    "ylim": (-1, 5),    "color": "k", "data": data_c_err},
    {"title": "Body Err Flag",       "ylim": (-1, 5),    "color": "k", "data": data_b_err},

    # Row 4: Alive Counters (Debug)
    {"title": "Vision Alive (0-15)", "ylim": (0, 16),    "color": "c", "data": data_v_alive},
    {"title": "Chassis Alive (0-15)","ylim": (0, 16),    "color": "c", "data": data_c_alive},
    {"title": "Body Alive (0-15)",   "ylim": (0, 16),    "color": "c", "data": data_b_alive},
    {"title": "Unused / Reserved",   "ylim": (0, 1),     "color": "w", "data": None} # 빈칸
]

def init_graph():
    for i, ax in enumerate(axs):
        config = plots_config[i]
        
        # 빈 칸 처리
        if config["data"] is None:
            ax.axis('off')
            lines.append(None)
            continue
            
        ax.set_title(config["title"], fontsize=9)
        ax.set_ylim(config["ylim"])
        ax.set_xlim(0, MAX_LEN)
        ax.grid(True, linestyle='--', alpha=0.5)
        
        # 라인 생성 및 리스트에 추가
        line, = ax.plot([], [], color=config["color"], lw=1.5)
        lines.append(line)
        
    return lines

def update(frame):
    for i, line in enumerate(lines):
        if line is None: continue
        
        # 설정에서 데이터 가져오기
        dq = plots_config[i]["data"]
        y_data = list(dq)
        x_data = range(len(y_data))
        
        line.set_data(x_data, y_data)
        
    return lines

# ==========================================
# [4] 실행
# ==========================================
ani = animation.FuncAnimation(fig, update, init_func=init_graph, 
                              interval=100, blit=False)

plt.tight_layout()
plt.show()