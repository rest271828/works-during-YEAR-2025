THRESHOLD = (0, 23, -128, 127, -128, 127)  # Grayscale threshold for dark things...
import sensor, image, time
from pyb import LED
# from machine import UART
import struct
# import PID
from pyb import UART
###############################
from pyb import Pin
from pid import PID

pin0 = Pin('P0', Pin.IN, Pin.PULL_UP)
pin6 = Pin('P6', Pin.IN, Pin.PULL_DOWN)

pin1 = Pin('P1', Pin.OUT, Pin.PULL_NONE)
pin2 = Pin('P2', Pin.OUT, Pin.PULL_NONE)

pin8 = Pin('P8', Pin.IN, Pin.PULL_DOWN)

# 控制灯的颜色，方便判断在task01还是task02
red_led = LED(1)  # 1代表红色LED
blue_led = LED(3)  # 3代表蓝色LED
green_led = LED(2) # 2代表蓝色

RX = 9  # 示例值，请根据实际情况调整
RY = 11  # 示例值，请根据实际情况调整
##########################################################################################################################################################
forward_speed = 440  # 最佳参数

NEI_HUAN_Speed = 500  # 内环参数

vy_pid = PID(p=17, i=0.001, d=0.06, imax=4)
# w_pid = PID(p=0.6, i=0, d=0, imax=5)
#################################################################################################################################################################
#w_pid = PID(p=0.518, i=0.1, d=0, imax=5)  # 当前最佳

color_pid = PID(p=5, i=0.1, d=0.1, imax=500)
vertical_pid = PID(p=5, i=0.1, d=0.1, imax=500)#识别颜色的pid

# ###########################################################################################################################################################
# sensor.reset()
# sensor.set_vflip(False)   # 设置OpenMV图像“水平方向进行翻转”
# sensor.set_hmirror(False) # 设置OpenMV图像“竖直方向进行翻转”

# #pid = PID
# sensor.set_pixformat(sensor.RGB565)
# sensor.set_framesize(sensor.QQQVGA) # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
# 线性回归算法的运算量大，越小的分辨率识别的效果越好，运算速度越快

# sensor.set_windowing([0,20,80,40])
# sensor.skip_frames(time = 2000)     # WARNING: If you use QQVGA it may take seconds
# clock = time.clock()                # to process a frame sometimes.
uart = UART(3, 115200)
# UART(1)是P0-RX P1-TX
uart.init(115200, bits=8, parity=None, stop=1)  # 8位数据位，无校验位，1位停止位

#######################################################

def Magnam_forward(VX, VY, omega):
    # 计算每个电机的速度
    V1 = VX - VY - omega * (RX + RY)
    V2 = VX + VY + omega * (RX + RY)
    V3 = VX + VY - omega * (RX + RY)
    V4 = VX - VY + omega * (RX + RY)

    if V1 >= 1000:
        V1 = 1000
    elif V1 < -1000:
        V1 = -1000

    # 对V2进行限幅
    if V2 >= 1000:
        V2 = 1000
    elif V2 < -1000:
        V2 = -1000

    # 对V3进行限幅
    if V3 >= 1000:
        V3 = 1000
    elif V3 < -1000:
        V3 = -1000

    # 对V4进行限幅
    if V4 >= 1000:
        V4 = 1000
    elif V4 < -1000:
        V4 = -1000

    # 创建并发送控制命令
    send_command(6, 1500 + V1)
    send_command(7, 1500 - V2)
    send_command(8, 1500 + V3)
    send_command(9, 1500 - V4)

def send_command(motor_id, position):
    # 格式化命令字符串
    command = "#{:03d}P{:04d}T0000!".format(motor_id, int(position))
    # 通过串口发送命令
    uart.write(command + '\r\n')
    # 等待一段时间，确保命令被处理
    time.sleep(0.01)

#######################################################

def send_color_packet(color_, robot_err_, shuiping_err_):
    color = int(color_)
    robot_err = int(robot_err_)
    shuiping = int(shuiping_err_)
    temp = struct.pack(
        ">biiib",
        0x5A,
        color,
        robot_err,
        shuiping,
        0xFE
    )  # 专门发颜色的包，会发送水平位移差，和差值的方向
    uart.write(temp)
    time.sleep_ms(100)

# def send_data_packet(pingyi_zhongxin_, bool_pingyi_, angle_, bool_angle):
#     pingyi_zhongxin = int(pingyi_zhongxin_)
#     bool_pingyi = int(bool_pingyi_)
#     angle = int(angle_)
#     bool_angle = int(bool_angle)
#     temp = struct.pack(">biiiib",                #格式为小端模式俩个字符俩个整型
#                    0x5A,                       #帧头1
#                   # 0xBB                       #帧头2
#                    pingyi_zhongxin,# up sample by 4    #数据1
#                    bool_pingyi,   #现在在中心的左边还是右边     #数据2
#                    angle,          #旋转角度
#                    bool_angle,
#                    0xFE)    #向左转还是向右转
#     uart.write(temp)
#     time.sleep_ms(100)
#    time.sleep(0.05)
# 串口发送
######################################################################################
# def task03():#什么也不干函数
#     sensor.reset()
#     sensor.set_vflip(False)             # 设置OpenMV图像“水平方向进行翻转”
#     sensor.set_hmirror(False)           # 设置OpenMV图像“竖直方向进行翻转”
#     sensor.set_pixformat(sensor.RGB565)
#     sensor.set_framesize(sensor.QVGA)   # 设置分辨率为QVGA（320x240）

#     clock = time.clock()                # 初始化时钟

#     blue_led.off()
#     red_led.off()
#     green_led.on()

#     while(True):
#         Magnam_forward(0,0,0)

######################################################################################
def task01():

    sensor.set_framesize(sensor.QQQVGA)  # 80x60 (4,800 pixels) - O(N^2) max = 2,3040,000.
    # 线性回归算法的运算量大，越小的分辨率识别的效果越好，运算速度越快

    clock = time.clock()  # to process a frame sometimes.
    state = 0  # 当前状态

    while (True):
        if pin0.value() == 0 and pin6.value() == 1 or pin0.value() == 0 and pin6.value() == 0 or pin0.value() == 1 and pin6.value() == 1:
            break  # 跳出循环
        blue_led.off()
        green_led.off()
        red_led.on()

        if pin8.value() == 0 and state == 0:
                forward_speed = 400
                w_pid = PID(p=0.515, i=0.1, d=0, imax=5)
                #vy_pid = PID(p=17, i=0.001, d=0.06, imax=4)
        elif pin8.value() == 1 and state == 0:  # 第一次为1
                forward_speed = 630
                state = 1  # 状态二
                w_pid = PID(p=0.520, i=0.01, d=0, imax=5)
                #vy_pid = PID(p=17, i=0.001, d=0.06, imax=4)
        elif pin8.value() == 0 and state == 1:  # 第二次为0
                forward_speed = 430
                w_pid = PID(p=0.515, i=0.1, d=0, imax=5)
                #vy_pid = PID(p=17, i=0.05, d=0.06, imax=100)

        #由于场地过长，为了减少市场，增大不同段的速度，三段路径不同的速度适配不同的PID
        print("task01 1111111111111111")
        print(w_pid._kp)
        clock.tick()
        img = sensor.snapshot().binary([THRESHOLD])

        img_width = img.width()
        img_height = img.height()
        black_area_height = img_height // 6+1  #####滤除图像中的干扰部分
        img.draw_rectangle(0, img_height - black_area_height, img_width, black_area_height, color=0, fill=True)

        line = img.get_regression([(100, 100)], robust=True)  # 调用线性回归函数
        # 对所有的阈值像素进行线性回归
        # 线性回归的效果就是将我们视野中“二值化”分割后的图像回归成一条直线
        if (line):
            ##############################################################
            delta_error = 0
            delta_error = abs(line.rho()) - img.width() / 2

            angle_error = 0

            angle_error = line.theta()
            if angle_error > 90:
                angle_error = angle_error - 180

            angle_error = -angle_error
            delta_error = -delta_error
            if delta_error > 30:
                delta_error = 15
            if delta_error < -30:
                delta_error = -15
            #print(delta_error)

            ##############################################################
            vy_output = vy_pid.get_pid(delta_error, 1)
            w_output = w_pid.get_pid(angle_error, 1)

            Magnam_forward(forward_speed, vy_output, w_output)

            #####################
            pingyi = 0
            rho_err = abs(line.rho()) - img.width() / 2
            if rho_err > 0:
                pingyi = 1
            else:
                pingyi = 0
                rho_err = abs(rho_err)
            # 计算我们的直线相对于中央位置偏移的距离（偏移的像素）
            # abs()函数：返回数字的绝对值  line.rho()：返回霍夫变换后的直线p值。

            ########################
            value = 0
            if line.theta() > 90:
                theta_err = abs(line.theta() - 180)  # 计算这条线距离垂直的角度
            else:
                theta_err = line.theta()  # 计算这条线距离垂直的角度
                value = 1  # 右拐
            #########################
            # 进行坐标的变换：y轴方向为0°，x轴正方向为90°，x轴负方向为-90°
            #    line_zhengfu = int(value)
            img.draw_line(line.line(), color=127)


def task02():
    sensor.set_framesize(sensor.QVGA)  # 设置分辨率为QVGA（320x240）
    # sensor.set_windowing((10, 0, 60, 60))  # 限制为 60x60 的窗口
    clock = time.clock()  # 初始化时钟

    # 定义红色、绿色、蓝色的阈值范围
    red_threshold = (30, 100, 15, 70, 15, 70)
    green_threshold = (17, 100, -128, -25, -120, 127)
    blue_threshold = (17, 100, -46, 19, -120, -16)

    color_new = 0  # 初始化颜色标识
    cross_x = 0  # 初始化十字中心点x坐标
    cross_y = 0  # 初始化十字中
    shuiping_err = 1  # 初始化为右拐的错误（默认为右拐）
    #############################################################################
    color_err = 0  # 颜色块关于中心点的误差
    vertical_err = 0

    while (True):
        if pin0.value() == 1 and pin6.value() == 0 or pin0.value() == 0 and pin6.value() == 0 or pin0.value() == 1 and pin6.value() == 1:
            break  # 跳出循环

        red_led.off()
        green_led.off()
        blue_led.on()
        print("task02 22222222222222222")
        img = sensor.snapshot()  # 获取一帧图像

        # 查找红色、绿色和蓝色的色块
        red_blobs = img.find_blobs([red_threshold], area_threshold=150, merge=True)
        green_blobs = img.find_blobs([green_threshold], area_threshold=150, merge=True)
        blue_blobs = img.find_blobs([blue_threshold], area_threshold=150, merge=True)

        # 默认没有识别到颜色
        color_new = 0
        shuiping_err = 1  # 默认右拐
        robot_err = 0

#(0, 49, -30, 0, 0, 22)黑色阈值

        # 检查是否有红色色块
        if red_blobs:
            for blob in red_blobs:
                img.draw_rectangle(blob.rect(), color=(0, 0, 0))  # 红色框
                img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 0))  # 红色十字
                img.draw_string(blob.cx(), blob.cy(), "Red: {}".format(blob.area()), color=(255, 255, 255))  # 显示面积
                cross_x = blob.cx()  # 记录红色区域的中心点x坐标
                cross_y = blob.cy()
                color_new = 4  # 设定为红色
                pin1.value(1)
                pin2.value(1)
                break  # 一旦识别到红色，跳出循环，不再处理其他颜色

        # 如果没有红色，则检查绿色
        if color_new == 0 and green_blobs:
            for blob in green_blobs:
                img.draw_rectangle(blob.rect(), color=(0, 0, 0))  # 绿色框
                img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 0))  # 绿色十字
                img.draw_string(blob.cx(), blob.cy(), "Green: {}".format(blob.area()), color=(255, 255, 255))  # 显示面积
                cross_x = blob.cx()  # 记录绿色区域的中心点x坐标
                cross_y = blob.cy()
                color_new = 3  # 设定为绿色
                pin1.value(0)
                pin2.value(1)
                break  # 一旦识别到绿色，跳出循环，不再处理其他颜色

        # 如果没有红色和绿色，则检查蓝色
        if color_new == 0 and blue_blobs:
            for blob in blue_blobs:
                img.draw_rectangle(blob.rect(), color=(0, 0, 0))  # 蓝色框
                img.draw_cross(blob.cx(), blob.cy(), color=(0, 0, 0))  # 蓝色十字
                img.draw_string(blob.cx(), blob.cy(), "Blue: {}".format(blob.area()), color=(255, 255, 255))  # 显示面积
                cross_x = blob.cx()  # 记录蓝色区域的中心点x坐标
                cross_y = blob.cy()
                color_new = 2  # 设定为蓝色
                pin1.value(0)
                pin2.value(0)
                break  # 一旦识别到蓝色，跳出循环

        # 如果识别到颜色，计算水平偏移
        if cross_x:
            robot_err = abs(cross_x) - img.width() / 2
            if robot_err > 0:
                shuiping_err = 1  # 右拐
            else:
                shuiping_err = 0  # 左拐

            robot_err = abs(robot_err)  # 获取无符号的误差值

        # 延时控制，避免过高的帧率导致过载
        time.sleep(0.01)  # 调整为更低的延时，使得视频更加流畅

        # 如果没有检测到任何色块，执行默认操作
        if not red_blobs and not green_blobs and not blue_blobs:
            Magnam_forward(-200, 0, -1)  # 无色块时执行默认移动
        else:
            color_err = 0
            color_err = -(abs(cross_x) - img.width() / 2)
            color_ouput = color_pid.get_pid(color_err, 1)

            vertical_err = 0
            vertical_err = cross_y - 85
            vertical_output = vertical_pid.get_pid(vertical_err, 1)

            Magnam_forward(color_ouput, vertical_output, 0)  # 根据 PID 控制移动

        ###############################################################################################
        print(color_err,vertical_err)

def task03():  # 新的什么也不干函数
    #Magnam_forward(0, 0, 0)
    print("task03 3333333333333333.")
    blue_led.off()
    red_led.off()
    green_led.on()

def task04():
    sensor.set_framesize(sensor.QVGA)
    black_threshold = (0, 30, -30, 0, 0, 22)

    while True:
        if pin0.value() == 1 and pin6.value() == 0 or pin0.value() == 0 and pin6.value() == 0 or pin0.value() == 0 and pin6.value() == 0:
            break
        blue_led.on()
        red_led.on()
        green_led.on()
        img = sensor.snapshot()  # 捕获图像
        black_blobs = img.find_blobs([black_threshold], area_threshold=150, merge=True)

        color_new = 5

        # 如果找到黑色区域，则寻找面积最大的区域
        if black_blobs:
            max_blob = max(black_blobs, key=lambda b: b.area())  # 找到面积最大的 Blob

            # 绘制最大 Blob 的矩形框和中心点
            img.draw_rectangle(max_blob.rect(), color=(0, 0, 0))  # 红色框
            img.draw_cross(max_blob.cx(), max_blob.cy(), color=(255, 255, 255))  # 白色十字
            img.draw_string(max_blob.cx(), max_blob.cy(), "Black: {}".format(max_blob.area()), color=(255, 255, 255))  # 显示面积

            # 记录最大区域的中心点
            cross_x = max_blob.cx()
            cross_y = max_blob.cy()

            if cross_x:
                robot_err = abs(cross_x) - img.width() / 2
                if robot_err > 0:
                    shuiping_err = 1  # 右拐
                else:
                    shuiping_err = 0  # 左拐

                robot_err = abs(robot_err)  # 获取无符号的误差值

            # 延时控制，避免过高的帧率导致过载
            time.sleep(0.01)  # 调整为更低的延时，使得视频更加流畅

            # 如果没有检测到任何色块，执行默认操作
            if not black_blobs or max_blob.w()<140:
                Magnam_forward(-250, 0, -1)  # 无色块时执行默认移动
            else:
                color_err = 0
                color_err = -(abs(cross_x) - img.width() / 2)-90
                color_ouput = color_pid.get_pid(color_err, 1)

                vertical_err = 0
                vertical_err = cross_y - 120
                vertical_output = vertical_pid.get_pid(vertical_err, 1)

                Magnam_forward(color_err, vertical_err, 0)  # 根据 PID 控制移动

            ###############################################################################################
            print("task04 44444444444444444444.")


def main():
    sensor.reset()
    sensor.set_vflip(False)  # 设置OpenMV图像“水平方向进行翻转”
    sensor.set_hmirror(False)  # 设置OpenMV图像“竖直方向进行翻转”
    sensor.set_pixformat(sensor.RGB565)

    while True:
        if pin0.value() == 1 and pin6.value() == 0:  # 默认执行巡线，not pin.value()会执行颜色识别
            blue_led.off()
            task01()  # 执行任务一（黑线识别：线性回归）
        elif pin0.value() == 0 and pin6.value() == 1:
            red_led.off()
            task02()  # 执行任务二（颜色识别+pid调节底盘控制自身的位置）
        elif pin0.value() == 0 and pin6.value() == 0:
            task03()  # 执行任务三（啥都不做）purpose:防止在识别颜色过程中openmv与arduino争夺大脑控制权
        elif pin0.value() == 1 and pin6.value() == 1:
            task04()  # 执行任务四（把黑色的块放到黑色的台子上，黑色识别并框选最大黑色+pid控制底盘修改位置）

if __name__ == "__main__":
    main()
