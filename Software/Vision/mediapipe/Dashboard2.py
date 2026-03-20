import socket
import struct
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib.gridspec as gridspec
from collections import deque
import threading
import numpy as np

# ==========================================
# [1] 데이터 수신 설정 (UDP)
# ==========================================
UDP_IP = "127.0.0.1"
UDP_PORT = 5005
MAX_LEN = 100

# 데이터 저장소 (Deque)
data_store = {
    "perclos": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "risk": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "steer": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "hands": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "head": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "noop": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "st_std": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    
    # Boolean / Status Data
    "eye": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "face": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "v_err": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "c_err": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "b_err": deque([0]*MAX_LEN, maxlen=MAX_LEN),
    "v_alive": deque([0]*MAX_LEN, maxlen=MAX_LEN),
}

FMT = '<H 5B 2f 2B 3f 2B B' # 32 bytes

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Dashboard Listening on {UDP_PORT}...")
    
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            if len(data) == 32:
                p = struct.unpack(FMT, data)
                # p[0]=Header
                data_store["perclos"].append(p[1])
                data_store["eye"].append(p[2])
                data_store["face"].append(p[3])
                data_store["v_alive"].append(p[4])
                data_store["v_err"].append(p[5])
                
                data_store["st_std"].append(p[6])
                data_store["steer"].append(p[7])
                data_store["c_err"].append(p[9])

                data_store["head"].append(p[10])
                data_store["hands"].append(p[11])
                data_store["noop"].append(p[12])
                data_store["b_err"].append(p[14])
                
                data_store["risk"].append(p[15])
        except:
            pass

t = threading.Thread(target=udp_listener, daemon=True)
t.start()

# ==========================================
# [2] 디자인 스타일 함수
# ==========================================
def style_ax(ax, title=""):
    ax.set_facecolor('white')
    ax.set_title(title, loc='left', pad=15, fontsize=14, fontweight='bold', color='#444')
    for spine in ['top', 'right']:
        ax.spines[spine].set_visible(False)
    ax.spines['left'].set_color('#ddd')
    ax.spines['bottom'].set_color('#ddd')
    ax.tick_params(axis='both', colors='#888', length=0, labelsize=9)
    ax.grid(axis='y', linestyle='--', alpha=0.3)

# ==========================================
# [3] 레이아웃 및 그래프 초기화
# ==========================================
fig = plt.figure(figsize=(16, 9), facecolor='#ebeff2')
gs = gridspec.GridSpec(2, 3, width_ratios=[1, 1, 1], height_ratios=[1, 1])
gs.update(wspace=0.2, hspace=0.25, left=0.05, right=0.95, top=0.9, bottom=0.05)

# --- [A] 상단 왼쪽: Area Chart ---
ax1 = plt.subplot(gs[0, 0:2])
style_ax(ax1, "Integrated Risk Monitor")
ax1.set_ylim(0, 100)
x_vals = np.arange(MAX_LEN)

line_risk, = ax1.plot([], [], color='#ff9f1a', linewidth=2.5, label='Risk Score')
line_perclos, = ax1.plot([], [], color='#0052d4', linewidth=1.5, linestyle='--', label='PERCLOS %')
ax1.legend(frameon=False, loc='upper left', ncol=2)

# --- [B] 상단 오른쪽: KPI Card ---
ax2 = plt.subplot(gs[0, 2])
ax2.set_facecolor('#6a11cb')
ax2.set_xticks([])
ax2.set_yticks([])
for spine in ax2.spines.values(): spine.set_visible(False)

text_kpi_val = ax2.text(0.5, 0.75, 'WAIT', ha='center', va='center', color='white', fontsize=60, fontweight='bold')
text_kpi_label = ax2.text(0.5, 0.60, 'RISK LEVEL', ha='center', va='center', color='white', fontsize=12, alpha=0.8)

status_texts = []
labels = ["FACE DETECT", "EYE STATE", "HANDS OFF", "SYS ERROR"]
y_pos = [0.4, 0.3, 0.2, 0.1]
for i, label in enumerate(labels):
    ax2.text(0.1, y_pos[i], label, ha='left', va='center', color='white', fontsize=10, alpha=0.7)
    t_val = ax2.text(0.9, y_pos[i], "-", ha='right', va='center', color='white', fontsize=10, fontweight='bold')
    status_texts.append(t_val)

# --- [C] 하단 왼쪽: Donut Chart ---
ax3 = plt.subplot(gs[1, 0])
style_ax(ax3, "Attention Ratio (PERCLOS)")
ax3.axis('off')
# 텍스트만 미리 생성 (파이는 update에서 매번 그림)
text_donut_center = None 

# --- [D] 하단 오른쪽: Line Chart ---
ax4 = plt.subplot(gs[1, 1:3])
style_ax(ax4, "Vehicle Dynamics")
ax4.set_ylim(-100, 100)
line_steer, = ax4.plot([], [], label='Steering Angle', color='#4facfe', linewidth=2)
line_head, = ax4.plot([], [], label='Head Delta', color='#00f260', linewidth=2)
line_hands, = ax4.plot([], [], label='Hands Off (x10)', color='#ffb347', linewidth=2, linestyle=':')
ax4.legend(frameon=False, loc='upper right')

# ==========================================
# [4] 애니메이션 업데이트
# ==========================================
def update(frame):
    # 데이터가 아직 없으면 대기
    if len(data_store["risk"]) < 1: 
        return

    # 1. 데이터 가져오기
    risk = list(data_store["risk"])
    perclos = list(data_store["perclos"])
    steer = list(data_store["steer"])
    head = list(data_store["head"])
    hands = list(data_store["hands"])
    
    curr_risk = risk[-1]
    curr_perclos = perclos[-1]

    # --- Update [A] Area Chart ---
    # [수정된 부분] AttributeError 해결을 위해 list()로 감싸서 하나씩 지움
    for c in list(ax1.collections):
        c.remove()
        
    ax1.fill_between(x_vals[:len(risk)], risk, color='#ffb347', alpha=0.4)
    ax1.fill_between(x_vals[:len(perclos)], perclos, color='#4facfe', alpha=0.2)
    
    line_risk.set_data(x_vals[:len(risk)], risk)
    line_perclos.set_data(x_vals[:len(perclos)], perclos)
    ax1.set_xlim(0, MAX_LEN)

    # --- Update [B] KPI Card ---
    text_kpi_val.set_text(f"{curr_risk}")
    
    if curr_risk > 80: ax2.set_facecolor('#ff4444') 
    elif curr_risk > 50: ax2.set_facecolor('#ffb347')
    else: ax2.set_facecolor('#6a11cb')
    
    is_face = data_store["face"][-1]
    is_eye_closed = data_store["eye"][-1]
    hands_sec = data_store["hands"][-1]
    err_sum = data_store["v_err"][-1] + data_store["c_err"][-1] + data_store["b_err"][-1]
    
    status_texts[0].set_text("YES" if is_face else "NO")
    status_texts[1].set_text("CLOSED" if is_eye_closed else "OPEN")
    status_texts[2].set_text(f"{hands_sec:.1f} sec")
    status_texts[3].set_text("FAIL" if err_sum > 0 else "OK")
    
    # --- Update [C] Donut Chart ---
    ax3.clear()
    # clear() 후에는 스타일이 날아가므로 다시 꺼줌
    ax3.axis('off')
    ax3.set_title("Attention Ratio (PERCLOS)", loc='left', pad=15, fontsize=14, fontweight='bold', color='#444')
    
    sizes = [curr_perclos, 100 - curr_perclos]
    colors = ['#4facfe', '#eee']
    if curr_perclos > 30: colors[0] = '#ff4444'
    
    ax3.pie(sizes, colors=colors, startangle=90, counterclock=False, 
            wedgeprops=dict(width=0.3, edgecolor='w'))
    ax3.text(0, 0, f"{curr_perclos}%", ha='center', va='center', fontsize=28, fontweight='bold', color='#444')
    
    # --- Update [D] Line Chart ---
    hands_scaled = [h * 10 for h in hands]
    
    line_steer.set_data(x_vals[:len(steer)], steer)
    line_head.set_data(x_vals[:len(head)], head)
    line_hands.set_data(x_vals[:len(hands)], hands_scaled)
    
    min_val = min(min(steer), -100)
    max_val = max(max(steer), 100)
    ax4.set_ylim(min_val - 20, max_val + 20)
    ax4.set_xlim(0, MAX_LEN)

# [수정된 부분] cache_frame_data=False 추가 (UserWarning 해결)
ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)

plt.show()