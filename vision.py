# OpenMV 视觉识别代码 - 固定平面方案
# 功能：颜色识别 + 坐标滤波 + 串口发送 + 握手等待

import sensor, image, time, math
import ustruct
from pyb import UART, LED

# ==================== 用户配置区 ====================

# 颜色阈值 (LAB格式: L_min, L_max, A_min, A_max, B_min, B_max)
# 根据实际环境调整，可用OpenMV IDE的工具->机器视觉->阈值编辑器获取
COLOR_THRESHOLDS = {
    'blue':   (73, 87, -65, 12, -44, 0),      # 蓝色
    'white':  (90, 100, -12, 14, -6, 26), # 白色
    'red':    (49, 77, 17, 62, -7, 62),       # 红色
}

# 识别参数
MIN_PIXELS = 200        # 最小色块像素，过滤噪声
MIN_AREA = 200          # 最小色块面积

# 滤波参数
HISTORY_SIZE = 5        # 移动平均窗口大小
STABLE_THRESHOLD = 15   # 稳定性判断阈值（像素方差）
STABLE_COUNT = 3        # 连续稳定帧数才发送

# 帧率控制
SEND_INTERVAL = 300     # 发送间隔(ms)，防止Arduino处理不过来

# ==================== 初始化 ====================

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)           # 320x240
#sensor.set_hmirror(True)                    #水平方向翻转
sensor.set_vflip(True)                      #垂直方向翻转
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)                 # 关闭自动增益，颜色更稳定
sensor.set_auto_whitebal(False)             # 关闭自动白平衡

# 串口初始化 (UART3: P4=P5, 波特率9600)
uart = UART(3, 9600)
uart.init(9600, bits=8, parity=None, stop=1)

# LED指示
led_red = LED(1)
led_green = LED(2)
led_blue = LED(3)

clock = time.clock()

# 全局状态
history = []            # 坐标历史
stable_counter = 0      # 稳定计数器
last_send_time = 0      # 上次发送时间
waiting_ack = False     # 是否等待Arduino确认

# ==================== 函数定义 ====================

def find_largest_blob(blobs):
    """找出最大的色块"""
    if not blobs:
        return None
    return max(blobs, key=lambda b: b.pixels())

def update_history(cx, cy):
    """更新坐标历史，返回是否稳定"""
    global history, stable_counter

    history.append((cx, cy))
    if len(history) > HISTORY_SIZE:
        history.pop(0)

    # 计算移动平均
    avg_x = sum(p[0] for p in history) / len(history)
    avg_y = sum(p[1] for p in history) / len(history)

    # 计算方差判断稳定性
    if len(history) >= 3:
        variance = sum(((p[0]-avg_x)**2 + (p[1]-avg_y)**2) for p in history) / len(history)
        is_stable = variance < STABLE_THRESHOLD**2
    else:
        is_stable = False

    # 稳定计数
    if is_stable:
        stable_counter += 1
    else:
        stable_counter = 0

    return int(avg_x), int(avg_y), is_stable

def clear_history():
    """清空历史记录"""
    global history, stable_counter
    history = []
    stable_counter = 0

def send_data(color_char, cx, cy):
    """发送数据包到Arduino"""
    global last_send_time

    try:
        # 数据包格式: 帧头0x2C, 0x12, 颜色, X高字节, X低字节, Y高字节, Y低字节, 帧尾0x5B
        # 共8字节，X和Y各用2字节(unsigned short)，解决坐标值超过255的问题
        data = ustruct.pack("<BBBHHB",
                          0x2C, 0x12,
                          ord(color_char),
                          int(cx), int(cy),
                          0x5B)
        uart.write(data)
        last_send_time = time.ticks_ms()

        # 调试输出
        print(f"[SEND] 颜色:{color_char} 像素:({cx},{cy})")
        return True
    except Exception as e:
        print(f"[ERROR]发送失败: {e}")
        return False

def check_ack():
    """检查Arduino的[READY]信号"""
    if uart.any():
        msg = uart.read()
        if msg and b'[READY]' in msg:
            print("[ACK] 收到Arduino就绪信号")
            return True
    return False

def draw_detection(img, blob, color_name, color_rgb):
    """绘制识别结果"""
    if blob:
        img.draw_rectangle(blob.rect(), color=color_rgb)
        img.draw_cross(blob.cx(), blob.cy(), size=5, color=color_rgb, thickness=2)
        img.draw_string(blob.x(), blob.y()-12, color_name, color=color_rgb, scale=1.5)

# ==================== 主循环 ====================

print("=" * 40)
print("OpenMV 机械臂视觉系统启动")
print("分辨率: QVGA (320x240)")
print("等待Arduino连接...")
print("=" * 40)

# 启动握手：等待Arduino发送初始[READY]信号
while True:
    if uart.any():
        msg = uart.read()
        if msg and b'[READY]' in msg:
            print("[ACK] 收到Arduino初始就绪信号")
            led_green.on()
            time.sleep_ms(500)
            led_green.off()
            break
    time.sleep_ms(100)

print("通信已建立，开始视觉识别...")
print("=" * 40)

while True:
    clock.tick()
    current_time = time.ticks_ms()

    # 拍摄图像并校正畸变
    img = sensor.snapshot().lens_corr(strength=1.8,zoom = 1.0)

    # 如果正在等待Arduino完成，检查确认信号
    if waiting_ack:
        led_blue.on()  # 蓝灯亮表示等待中

        if check_ack():
            waiting_ack = False
            clear_history()
            led_blue.off()
            led_green.on()
            time.sleep_ms(100)
            led_green.off()
            print("准备下一次识别...")
        else:
            # 显示等待状态
            img.draw_string(10, 10, "Wait Arduino...", color=(255,255,0), scale=2)

        continue  # 等待期间不进行新的检测

    # 正常识别流程
    led_blue.off()

    # 查找各色块
    blobs_blue = img.find_blobs([COLOR_THRESHOLDS['blue']],
                                pixels_threshold=MIN_PIXELS,
                                area_threshold=MIN_AREA)
    blobs_white = img.find_blobs([COLOR_THRESHOLDS['white']],
                                 pixels_threshold=MIN_PIXELS,
                                 area_threshold=MIN_AREA)
    blobs_red = img.find_blobs([COLOR_THRESHOLDS['red']],
                               pixels_threshold=MIN_PIXELS,
                               area_threshold=MIN_AREA)

    # 优先级: 蓝 -> 白 -> 红
    detected_blob = None
    color_name = ""
    color_char = 'A'
    color_rgb = (128, 128, 128)

    if blobs_blue:
        detected_blob = find_largest_blob(blobs_blue)
        color_name = "Blue"
        color_char = 'B'
        color_rgb = (0, 0, 255)

    elif blobs_white:
        detected_blob = find_largest_blob(blobs_white)
        color_name = "White"
        color_char = 'P'
        color_rgb = (255, 255, 255)

    elif blobs_red:
        detected_blob = find_largest_blob(blobs_red)
        color_name = "Red"
        color_char = 'R'
        color_rgb = (255, 0, 0)

    # 处理检测结果
    if detected_blob:
        cx, cy = detected_blob.cx(), detected_blob.cy()

        # 更新历史并检查稳定性
        avg_cx, avg_cy, is_stable = update_history(cx, cy)

        # 绘制实时位置（白色）和平均位置（彩色）
        img.draw_cross(cx, cy, size=3, color=(128,128,128), thickness=1)
        draw_detection(img, detected_blob, color_name, color_rgb)

        # 显示稳定性指示
        status_text = f"{color_name} ({stable_counter}/{STABLE_COUNT})"
        img.draw_string(10, 10, status_text, color=color_rgb, scale=1.5)

        # 稳定后才发送，且满足发送间隔
        time_since_last = time.ticks_diff(current_time, last_send_time)

        if stable_counter >= STABLE_COUNT and time_since_last > SEND_INTERVAL:
            # 发送平均坐标（更稳定）
            if send_data(color_char, avg_cx, avg_cy):
                waiting_ack = True  # 进入等待状态
                print(f"已发送{color_name}坐标({avg_cx},{avg_cy})，等待Arduino完成...")

    else:
        # 未检测到，清空历史
        clear_history()
        img.draw_string(10, 10, "No target", color=(128,128,128), scale=1.5)

    # 显示帧率（调试用，可注释）
    # img.draw_string(260, 220, f"{clock.fps():.0f}fps", color=(255,255,255), scale=1)
