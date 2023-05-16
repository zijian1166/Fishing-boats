import _thread
import time
import sys
import machine
from machine import Pin, SoftI2C, PWM
from time import sleep
import ssd1306
import utime

# 引脚定义 LED
pin_d2 = Pin(2, Pin.OUT)
# ---------- OLED 初始化 ------------------------
# 创建i2c对象
i2c = SoftI2C(scl=Pin(22), sda=Pin(21))

# 宽度高度
oled_width = 128
oled_height = 64
# 屏幕初始化失败标志位
OLED_err = 0

# 创建oled屏幕对象
# oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)

# ---------- OLED 初始化 ------------------------
i2c = SoftI2C(scl=Pin(22), sda=Pin(21))

# 宽度高度
oled_width = 128
oled_height = 64

# 创建 OLED 屏幕对象（添加异常处理）
try:
    oled = ssd1306.SSD1306_I2C(oled_width, oled_height, i2c)
except Exception as e:
    print("OLED 初始化失败:", str(e))
    OLED_err = 1

# -------------- uart1 初始化 ------------------
uart = machine.UART(1, baudrate=100000, tx=17, rx=16)
sbus_frame = bytearray(25)  # 存储完整的SBUS帧数据
frame_index = 0  # 当前接收到的数据帧索引
frame_count = 0  # 接收到的数据帧数量
MAX_FRAME_COUNT = 200  # 阈值，接收到一定数量的数据帧后重置
# 获取通道1数据
# channel1_data = 1036
# channel2_data = 1022
# channel3_data = 1028
# channel4_data = 1020
# channel5_data = 240
# channel6_data = 240


channel1_data = 0
channel2_data = 0
channel3_data = 0
channel4_data = 0
channel5_data = 0
channel6_data = 1807
channel7_data = 0
channel8_data = 0

# 船模式选择：
pattern = 0  # 模式0

# 开锁变量 0：锁住 1：开锁
open_boat = 0
# 判断遥控器是否连接
remote_control = 0
# 计步器
step = 0
i2 = 0


# ---------- 这是OLED & pattern 扫描 线程要执行的代码 ------------
def test1(*args, **kwargs):
    global channel1_data
    global open_boat
    global OLED_err
    while True:
        if channel5_data > 1800:
            open_boat = 1
        else:
            open_boat = 0
        if OLED_err == 0:
            # 清空屏幕
            oled.fill(0)
            # 在指定位置处显示文字
            # oled.text('1234', 0, 0)
            # oled.text('5678',40, 0)
            oled.text(str(channel1_data), 0, 0)
            oled.text(str(channel2_data), 40, 0)
            oled.text(str(channel3_data), 80, 0)
            oled.text(str(channel4_data), 0, 10)
            oled.text(str(channel5_data), 40, 10)
            oled.text(str(channel6_data), 80, 10)
            oled.text(str(channel7_data), 40, 20)
            oled.text(str(channel8_data), 80, 20)
            oled.text('open', 0, 30)
            oled.text(str(open_boat), 40, 30)

            oled.text('connect', 0, 40)
            oled.text(str(remote_control), 60, 40)
            oled.show()

        utime.sleep_ms(100)


# ---------- 这是SBUS线程要执行的代码 ------------
def test2(*args, **kwargs):
    global channel1_data
    global channel2_data
    global channel3_data
    global channel4_data
    global channel5_data
    global channel6_data
    global channel7_data
    global channel8_data
    global frame_index
    global channel1_data
    global frame_count
    global remote_control
    while True:
        if uart.any():
            data = uart.read(1)  # 读取一个字节的数据
        if frame_index == 0 and data != b'\x0F':  # 检查起始位
            continue

        sbus_frame[frame_index] = data[0]
        frame_index += 1

        if frame_index == 25:  # 完整接收到一帧SBUS数据
            frame_index = 0
            if sbus_frame[24] == 0x00:  # 检查停止位

                # 查看遥控器是否在线
                if sbus_frame[23] == 0x0C:  # 检查停止位
                    remote_control = 0
                else:
                    remote_control = 1

                # 解析通道数据
                channel_data = [
                    (sbus_frame[1] >> 0 | sbus_frame[2] << 8) & 0x7FF,
                    (sbus_frame[2] >> 3 | sbus_frame[3] << 5) & 0x7FF,
                    (sbus_frame[3] >> 6 | sbus_frame[4] << 2 | sbus_frame[5] << 10) & 0x7FF,
                    (sbus_frame[5] >> 1 | sbus_frame[6] << 7) & 0x7FF,
                    (sbus_frame[6] >> 4 | sbus_frame[7] << 4) & 0x7FF,
                    (sbus_frame[7] >> 7 | sbus_frame[8] << 1 | sbus_frame[9] << 9) & 0x7FF,
                    (sbus_frame[9] >> 2 | sbus_frame[10] << 6) & 0x7FF,
                    (sbus_frame[10] >> 5 | sbus_frame[11] << 3) & 0x7FF,
                    (sbus_frame[12] >> 0 | sbus_frame[13] << 8) & 0x7FF,
                    (sbus_frame[13] >> 3 | sbus_frame[14] << 5) & 0x7FF,
                    (sbus_frame[14] >> 6 | sbus_frame[15] << 2 | sbus_frame[16] << 10) & 0x7FF,
                    (sbus_frame[16] >> 1 | sbus_frame[17] << 7) & 0x7FF,
                    (sbus_frame[17] >> 4 | sbus_frame[18] << 4) & 0x7FF,
                    (sbus_frame[18] >> 7 | sbus_frame[19] << 1 | sbus_frame[20] << 9) & 0x7FF,
                    (sbus_frame[20] >> 2 | sbus_frame[21] << 6) & 0x7FF,
                    (sbus_frame[21] >> 5 | sbus_frame[22] << 3) & 0x7FF
                ]

                # 打印每个通道的值
                # print("通道数据：[{}]".format(", ".join(str(value) for value in channel_data)))
                channel1_data = channel_data[0]  # 获取通道1的值
                channel2_data = channel_data[1]  # 获取通道2的值
                channel3_data = channel_data[2]  # 获取通道3的值
                channel4_data = channel_data[3]  # 获取通道4的值
                channel5_data = channel_data[4]  # 获取通道5的值
                channel6_data = channel_data[5]  # 获取通道6的值
                channel7_data = channel_data[6]  # 获取通道7的值
                channel8_data = channel_data[7]  # 获取通道8的值

                frame_count += 1  # 数据帧计数器加1

                if frame_count >= MAX_FRAME_COUNT:
                    frame_count = 0  # 重置数据帧计数器
                    uart.read(1024)  # 清空接收缓冲区数据
            # else:
            # 错误的SBUS帧
            # print("错误的SBUS帧：", sbus_frame)
            # frame_index = 0  # 重置frame_index的值

            # print("")

    utime.sleep_us(100)  # 等待100微秒


# 查看遥控器线程
def test3(*args, **kwargs):
    while True:
        pin_d2.value(1)  # 输出高电平
        utime.sleep_ms(500)
        pin_d2.value(0)  # 输出高电平
        utime.sleep_ms(500)


# ---------- 这里创建线程 ------------
thread_1 = _thread.start_new_thread(test1, (1,))
thread_2 = _thread.start_new_thread(test2, (2,))
thread_3 = _thread.start_new_thread(test3, (3,))


# ---------- 通道映射函数 ------------
def map_calculate(value):
    value = value - 1028
    if value >= 0:
        return int(value * 0.12)
    else:
        return int(value * 0.12)


def calculate_motor_speed(channel3, channel4):
    # 将映射值转换为具体的速度范围
    speed_range = 94  # 速度范围（最大速度）
    max_speed = 100  # 最大速度

    # 计算左右电机的速度
    left_speed = 0
    right_speed = 0

    # 根据通道3和通道4的映射值计算左右电机的速度

    left_speed = int(channel3 / speed_range * max_speed)
    right_speed = int(channel3 / speed_range * max_speed)
    bias = int(channel4 / speed_range * max_speed * 0.3)

    if channel4 >= 0:  # 右转
        right_speed -= bias
        left_speed += bias
    else:  # 左转
        left_speed += bias
        right_speed -= bias

    if channel3 < 0:
        exchange = left_speed
        left_speed = right_speed
        right_speed = exchange

    if left_speed >= 100:
        left_speed = 100
    if left_speed <= -100:
        left_speed = -100
    if right_speed >= 100:
        right_speed = 100
    if right_speed <= -100:
        right_speed = -100

    # 返回左右电机的速度
    return left_speed, right_speed


# ---------- 这是主线程要执行的代码 ------------
# ---------- 这是主线程要执行的代码 ------------
left_speed = 0
right_speed = 0
c3 = 0
c4 = 0

# --------- PWM 测试 ------------------------
# 实例化对象
Pin25_pwm = PWM(Pin(25))
Pin26_pwm = PWM(Pin(26))

# 设置频率
Pin25_pwm.freq(50)
Pin26_pwm.freq(50)

# 初始化D18引脚 -> AIN1
pin_d18 = Pin(18, Pin.OUT)
pin_d18.value(0)  # 输出高电平

# 初始化D19引脚 -> AIN2
pin_d19 = Pin(19, Pin.OUT)
pin_d19.value(0)  # 输出高电平

# 初始化D12 -> BIN1
pin_d12 = Pin(12, Pin.OUT)
pin_d12.value(0)  # 输出高电平

# 初始化D13 -> BIN2
pin_d13 = Pin(13, Pin.OUT)
pin_d13.value(0)  # 输出高电平

# 初始化D5引脚 -> TB6612 使能
pin_d5 = Pin(5, Pin.OUT)
pin_d5.value(1)  # 输出高电平

while True:
    # time.sleep(1)
    utime.sleep_ms(100)
    # -------  step 1 ------------
    # 开机遥控器检测
    if step == 0:
        # --------------- 测试遥控器是否连接 -----------------
        if remote_control == 1:
            step = 1
        else:
            # 等待一段时间
            time.sleep(3)

            # 执行软件复位
            machine.reset()

    if step == 1:
        if remote_control == 1:
            c3 = map_calculate(channel3_data)
            c4 = map_calculate(channel4_data)
            left_speed, right_speed = calculate_motor_speed(c3, c4)
            # print(str(c4))
            print("left_speed：", left_speed)
            print("right_speed：", right_speed)
            # -------------  正转、反转 --------------------
            if left_speed >= 0:  # 左边电机
                pin_d18.value(0)  # 输出高电平
                pin_d19.value(1)  # 输出高电平
            else:
                pin_d18.value(1)  #
                pin_d19.value(0)  #

            if right_speed >= 0:  # 右边电机
                pin_d12.value(0)  #
                pin_d13.value(1)  #
            else:
                pin_d12.value(1)  #
                pin_d13.value(0)  #

            # 绝对值 PWM
            if left_speed < 0:
                left_speed = abs(left_speed)
            if right_speed < 0:
                right_speed = abs(right_speed)
            # 设置占空比
            left_speed = int(left_speed * 10)
            right_speed = int(right_speed * 10)

            # 限幅0 1023
            if left_speed >= 1023:
                left_speed = 1023
            if left_speed <= 0:
                left_speed = 0
            if right_speed >= 1023:
                right_speed = 1023
            if right_speed <= 0:
                right_speed = 0

            if open_boat == 1:
                left_speed = int(left_speed * 0.5)
                right_speed = int(right_speed * 0.5)
                Pin25_pwm.duty(left_speed)
                Pin26_pwm.duty(right_speed)
            else:
                Pin25_pwm.duty(0)
                Pin26_pwm.duty(0)
