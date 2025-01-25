
#include <Servo.h>
#include <math.h>
//110闭合
//可以调节的参数 以及重要函数 stop_and_get stop_and_put 这些函数中的延时;以及逻辑函数 time_count_2*****ctrl+F一键查找这些函数*******************
#define COOL_DOWN_TIME 10000     // 障碍物检测冷却时间 (单位：毫秒)
#define DETECT_DISTANCE 40      //障碍物触发距离
#define BACKTIME 0 //碰到障碍的后退时间 设置成多少任选
#define BACK_VELOCITY 0 //碰到障碍物后的速度 负数为后退

#define MEDIAN_SIZE 5 // 滤波窗口大小
int median_window[MEDIAN_SIZE] = {0};

unsigned long startTime = 0; // 开机时间
bool waitFor40Seconds = false; // 是否需要等待40秒

unsigned long now_time=0;
void change_speed(void);

#define WAIT_TIME 55000

void stop_and_put(void);

#define FIRST_SPEED_SWITCH 13000
#define SECOND_SPEED_SWITCH 47000

//********************************************************************

void state_change(void);
void W_control(void);

void have_a_look(void);//看一看
#define DO_NOTHING 1
#define COLOR_MODE 2
#define TRACK_MODE 3
#define BLACK_MODE 4

void VY_control(void);

void integrity_control(void);

void the_whole_task(void);

void volocity_PID_init(void);
void uart_receive_3(void);

int color_possess=0;
int level_erro=0;
int level_float=0;//关于物块的偏移正负

int now_angle=0;
int now_delta=0;

// 定义全局变量
int task_count = 0;         // 当前任务计数，反复三次
int ball_color = 2;        // 球的颜色信息
bool has_ball = false;      // 是否持有球
bool is_writing = false;    // 是否进入写字任务阶段

//一坨shit****************************************************************************
#include <SoftwareSerial.h>
#define POSTURE_DELAY 1000

int actual_distance=0;
int color_choose=0;

int color_recieve=3;
int color_middle=0;

// unsigned long curren_time=0;******************************************************************
unsigned long curren_time=0;
unsigned long last_time=0;
int new_count=0;
void time_count(void);
void arm_normal_type(void);
void arm_get(void);

int flag=0;
#define PACKET_SIZE 14
// SoftwareSerial SerialPort(A5, A4);//RX TX
int uart_what=0;
uint8_t packet[PACKET_SIZE]; // 用于存储接收到的数据包
void processPacket(uint8_t *data, size_t len);
bool validatePacket(uint8_t *data, size_t len);
int Vx,Vy,W=0;
int distance_count=0;
int obstacle_count=0;
#define DISTANCE_TOLERANCE 2
#define MIDDLE_DISTANCE 35
#define SEND_SIZE 6
// #define DETECT_DISTANCE 40/***************************************************************************************************************************************/
uint8_t dataToSend[SEND_SIZE] = {0};
void uart_send(uint8_t* dataArray, size_t length);
int color_this_round=0;

//一坨shit****************************************************************************

unsigned long lastAdjustTime = 0; // 上次调整时间

void put_block(void);//放置物块

void get_block(void);//拿走物块

void draw_ten(void);//画十

void draw_two(void);//画二

void draw_T(void);//画T

/****************机械臂控制***********************/
//舵机声明
Servo servo_7;
Servo servo_3;
Servo servo_5;
Servo servo_6;
Servo servo_9;
Servo servo_8;

//常量宏定义
#define ANGLE_ADJUST 10
#define POINT_NUM 12
#define GENERAL_ANGLE_DELAY 1
#define GENERAL_GESTURE_DELAY 100

//角度输出结构体
struct Angles_OUT {
    float theta1;
    float theta2;
    float theta3;
    float theta4;
};

//连杆参数
float L0=150;
float L1=105;
float L2=80;
float L3=180;

//弧度转角度
float to_deg(float angle);

//依据末端角度 找到第三关节坐标
float height_adjust(float z,float angle_adjust);
//依据末端角度 找到第三关节坐标
float length_adjust(float angle_adjust);
//反解出各个舵机角度
Angles_OUT backcalculate(float x,float y,float z);

//机械臂水平放置
void servo_init(void);

// 函数用于映射舵机序号到对应的Servo对象并应用运算逻辑
void controlServo(int servoNum, float coefficient);

//依据角度 整体控制机械臂
void control_arm(float a,float b,float c,float d,float e,float f);

//机械臂高举函数
void reset_arm(void);

//依据起点和终点xyz 绘制笔画
void draw_stroke(float begin_X,float begin_Y,float begin_Z,float end_X,float end_Y,float end_Z);

/****************颜色检测***********************/

//颜色检测变量
float color_detection=0;//颜色检测变量
#define NOTHING_DETECT 0
#define RED_DETECT 1
#define GREEN_DETECT 2
#define BLUE_DETECT 3

/****************PID函数***********************/
struct pid_struct_t
{
  float kp;//比例
  float ki;//积分
  float kd;//微分
  float i_max;//积分限幅
  float out_max;//输出限幅
  
  float ref;      // target value目标角度
  float fdb;      // feedback value设定角度
  float err[2];   // error and last error差值
 
  float p_out;//比例输出
  float i_out;//积分输出
  float d_out;//微分输出
  float output;//pid总输出
};
pid_struct_t distance_pid;

pid_struct_t W_pid;//转向PID
pid_struct_t VY_pid;//横移pid

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max);

float pid_calc(pid_struct_t *pid, float ref, float fdb);
void limit(float x,float min,float max);

/****************距离控制***********************/
void forward(int speed);
//距离参数初始化
void distance_init(void);
//检测距离
float checkdistance(void);
//开始距离控制
void start_distance_control(int target_distance);

//麦轮运动学解算变量
#define RX 9
#define RY 11
char cmd_return_tmp[64];
void Magnam_forward(float VX,float VY,float omega);

void setup(){
  Serial.begin(115200);
  servo_7.attach(7);
  servo_3.attach(3);
  servo_5.attach(5);
  servo_6.attach(6);
  servo_9.attach(9);
  servo_8.attach(8);

  distance_init();//超声波初始化

  Serial.println("shabi");

  servo_init();//给机械臂一个出生姿态
  
  pinMode(A1, INPUT);//颜色读取引脚
  pinMode(A2, INPUT);
 
  pinMode(2,OUTPUT);//控制openmv的引脚
  pinMode(4,OUTPUT); 
  pinMode(11,OUTPUT);
  // pinMode(12,OUTPUT);
 
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  pinMode(A5, INPUT);//两个状态切换引脚
  pinMode(A4, INPUT);

  control_arm(0,-45,0,0,5,0);
  delay(1000);
  arm_normal_type();//机械臂常态寻迹
  delay(1000);

  // startTime = millis(); // 记录开机时间
  // state_change();       // 调用状态切换函数

  set_openmv_state(TRACK_MODE);//让openmv寻迹 开跑！

  // digitalWrite(4,0);
  // stop_and_write(4);

  // digitalWrite(4,HIGH);
  // delay(2000);
  // set_openmv_state(BLACK_MODE);

  // delay(2000);
  // set_openmv_state(DO_NOTHING);//让openmv寻迹 开跑！

  // stop_and_put();

  // stop_and_put();
  // have_a_look();

  //*************************************************************
  // set_openmv_state(DO_NOTHING);//告诉openmv停止

  // // set_openmv_state(2);//识别颜色

  // forward(BACK_VELOCITY);
  // delay(BACKTIME);//后退 我不好说 依据需要
  // forward(0);

  // have_a_look();//机械臂观察颜色姿态
  // delay(2000);//等待机械臂稳定下来

  // set_openmv_state(COLOR_MODE);//开启颜色识别矫正
  // delay(6000);//等待上位机矫正底盘位置
  // color_digital_detect();//收集颜色信息 迭代ball_color
  // delay(1000);

  // set_openmv_state(DO_NOTHING);//告诉openmv停止 别抽风

  // delay(2000);
  // catch_it();//机械臂抓取
  // arm_normal_type();//机械臂复位
  // delay(1000);
//****************************************************************************

// stop_and_write(3);

  // set_openmv_state(DO_NOTHING);//告诉openmv停止
  
  // // Magnam_forward(-200, 0, -2);//这个可能需要上位机完成 我不好说 先保留
  // // delay(1000);//后退 我不好说
  // // forward(0);//停止 我不好说
  // delay(2000);
  // have_a_look();

  // delay(2000);
  // set_openmv_state(BLACK_MODE);

  // delay(6000);//等待openmv调整位置
  
  // set_openmv_state(DO_NOTHING);//告诉openmv停止
  // delay(2000);
  // arm_put();//随意抛弃你的球
  // delay(POSTURE_DELAY);
  // arm_normal_type();//复位机械臂位置
  // delay(POSTURE_DELAY);
    //   digitalWrite(4,HIGH);
    // digitalWrite(12,HIGH);
    // digitalWrite(4, LOW);
}

void loop(){
  // forward(200);
  // if(checkdistance()<=45)
  // {
  //     stop_and_write(3);
  //     delay(10000);
  // }
  change_speed();
  // set_openmv_state(TRACK_MODE);//让openmv寻迹 开跑！

  // if (waitFor40Seconds) {
  //   // 需要等待40秒的逻辑

    // if (millis() - startTime > WAIT_TIME) {
    //   digitalWrite(13, LOW);

    time_count_2(); // 40秒后开始执行time_count_2
    // } else {
    //   digitalWrite(13, HIGH);
    //   // 40秒内等待，可以添加其他逻辑
    // }

  // } else {
  //   // 不需要等待40秒的情况，直接执行
  //   time_count_2();
  // }

  // time_count_2();//超声波+决策

  // set_openmv_state(1);

  // set_openmv_state(2);
  // color_digital_detect();

}
 

float to_deg(float angle)//弧度转角度
{ return angle*180/PI ;}

float height_adjust(float z,float angle_adjust)//末端偏移
{
  float h=0;
  angle_adjust=angle_adjust*PI/180;//角度转弧度
  h=z+L3*sin(angle_adjust);//和水平面的角度
  
  return h;
}

float length_adjust(float angle_adjust)
{return L3*cos(angle_adjust*PI/180);}

Angles_OUT backcalculate(float x,float y,float z,int angle_adjust)
{ 
  
  Angles_OUT angles;
  // float L0=150;
  // float L1=105;
  // float L2=80;
  // float L3=180;
  float l1=0;
  
  float h0=0;
  
  float theta0=0;
  float theta1=0;
  float theta2=0;
  float theta3=0;
  float theta4=0;

  float x0=x;
  float y0=y;
  float z0=z-L0;//减去底座高度
  h0=height_adjust(z0,angle_adjust);//得到第三个关节的高度

  float l0 = sqrt(pow(x0, 2) + pow(y0, 2)) - length_adjust(angle_adjust);//调整第三个关节的水平位移
  //l1 = sqrt(pow(l1, 2) + pow(h0, 2));//得到第三个关节到原点的距离 此段代码存疑
  l1 = sqrt(pow(l0, 2) + pow(h0, 2));

  theta0=atan(h0/l0);//第三个关节关于原点的夹角，是中间量
  // theta3=PI-asin(l1*sin(theta2-theta0)/l2);

  angles.theta3 = PI - acos((pow(L1, 2) + pow(L2, 2) - pow(l1, 2)) / (2 * L1 * L2));//目前是锐角 不知其他的解怎么搞
  angles.theta2=(L2*sin(angles.theta3 )/l1)+theta0;//第一根杆的水平夹角
  angles.theta1=atan(y0/x0);//云台转动夹角

  angles.theta4=-(PI-angles.theta2-(PI-angles.theta3)-(angle_adjust*PI/180));

  // Serial.print(theta4*180/PI); 

  angles.theta1 = to_deg(angles.theta1);//全部转化为角度制
  angles.theta2 = to_deg(angles.theta2);
  angles.theta3 = to_deg(angles.theta3);
  angles.theta4 = to_deg(angles.theta4);

  Serial.print("云台转动夹角=");  
  Serial.println(angles.theta1);

  Serial.print("第一根杆的夹角=");
  Serial.println(angles.theta2);

  Serial.print("第二根杆的夹角=");  
  Serial.println(angles.theta3);

  Serial.print("第三根杆的夹角=");  
  Serial.println(angles.theta4);

  return angles;

}

void servo_init(void){
  servo_7.write(93);
  servo_3.write(120);
  servo_5.write(90);
  servo_6.write(90);
  servo_9.write(90);
  servo_8.write(0);
}

// 函数用于映射舵机序号到对应的Servo对象并应用运算逻辑
void controlServo(int servoNum, float coefficient) {
    Serial.print("Control Servo: ");
  Serial.print("servoNum: ");
  Serial.print(servoNum);  // 打印传入的舵机编号
  Serial.print(" coefficient: ");
  Serial.println(coefficient);  // 打印传入的系数
  // 根据舵机编号进行运算

  if (servoNum == 1) {
    int angle1 = (coefficient*2/3) + 93; 
    servo_7.write(constrain(angle1, 0, 180)); // 写入角度
  } 
  else if (servoNum == 2) {
      int angle2 = (coefficient*2/3) + 120; 
      servo_3.write(constrain(angle2, 0, 180));
      Serial.print(angle2);
  } 
  else if (servoNum == 3) {
      int angle3 = (coefficient*2/3) + 90; 
      servo_5.write(constrain(angle3, 0, 180));
  } 
  else if (servoNum == 4) {
      int angle4 = (coefficient*2/3) + 90;
      servo_6.write(constrain(angle4, 0, 180));
  } 
  else if (servoNum == 5) {
      int angle5 = (coefficient*2/3) + 90; 
      servo_9.write(constrain(angle5, 0, 180));
  } 
  else if (servoNum == 6) {
      int angle6 = (coefficient*2/3) + 0; 
      servo_8.write(constrain(angle6, 0, 180));
  } 
  else {
      Serial.println("Invalid servo number"); // 非法的舵机编号
  }

}

void control_arm(float a,float b,float c,float d,float e,float f){
  controlServo(1,a);
  delay(GENERAL_ANGLE_DELAY);
  controlServo(2,b);
  delay(GENERAL_ANGLE_DELAY);
  controlServo(3,c);
  delay(GENERAL_ANGLE_DELAY);
  controlServo(4,d);
  delay(GENERAL_ANGLE_DELAY);
  controlServo(5,e);
  delay(GENERAL_ANGLE_DELAY);
  controlServo(6,f);
  delay(GENERAL_ANGLE_DELAY);
}

void reset_arm(void)
{
  //机械臂复位
  control_arm(0,-135,90,45,0,0);
}

void draw_stroke(float begin_X,float begin_Y,float begin_Z,float end_X,float end_Y,float end_Z)//画横线
{
  float x[POINT_NUM], y[POINT_NUM], z[POINT_NUM];
  float point_begin[3] = {begin_X,begin_Y,begin_Z}; // 第一个点的坐标
  float point_end[3] = {end_X,end_Y,end_Z}; // 第二个点的坐标

  for (int i = 0; i < POINT_NUM; i++) {
    // 计算每个维度的插值 得到每个差值点的坐标
    x[i] = point_begin[0] + ((point_end[0] - point_begin[0]) * i / (POINT_NUM-1));
    y[i] = point_begin[1] + ((point_end[1] - point_begin[1]) * i / (POINT_NUM-1));
    z[i] = point_begin[2] + ((point_end[2] - point_begin[2]) * i / (POINT_NUM-1));

    x[0] = point_begin[0];
    y[0] = point_begin[1];
    z[0] = point_begin[2];

    x[POINT_NUM-1] = point_end[0];
    y[POINT_NUM-1] = point_end[1];
    z[POINT_NUM-1] = point_end[2];

  }

  for (int i = 0; i < POINT_NUM; i++) {//开始遍历所有点 准备控制

    bool valid = false;//将所有的点设置为无效
    int current_angle_adjust = ANGLE_ADJUST;//初始角度
    Angles_OUT angles;

    while (!valid) 
    {
      angles=backcalculate(int(x[i]), int(y[i]), int(z[i]),current_angle_adjust);//计算出当前的角度 //不对不对
      if (isnan(angles.theta1) || isnan(angles.theta2) || isnan(angles.theta3) || isnan(angles.theta4))//判断是否有解
      {
        if (current_angle_adjust > 0) {
          current_angle_adjust -= 1; // 减小偏移角度
          Serial.print("Invalid solution, reducing ANGLE_ADJUST to: ");//调整到一个角度
          Serial.println(current_angle_adjust);
        } 
        else {
          Serial.println("Cannot find valid solution, skipping this point.");
          break;//跳出while循环 舍弃这个点
        }
      }
      else //直到算出可行的解 如果可解 那么这个点有效
      {
        valid = true;
      }

    }
  // Angles_OUT angles = backcalculate(x[i], y[i], z[i]);
  // control_arm(angles.theta1,-angles.theta2,angles.theta3,angles.theta4,0,0);
  // delay(GENERAL_GESTURE_DELAY);

    if (valid) {//如果是一个有效的点
      Serial.print("Valid solution found with ANGLE_ADJUST: ");
      Serial.println(current_angle_adjust);
      control_arm(angles.theta1,-angles.theta2,angles.theta3,angles.theta4,0,0);//通过计算出的点 控制机械臂
      delay(GENERAL_GESTURE_DELAY);
    }
  }
  
  

}

void Magnam_forward(float VX,float VY,float omega)
{
  int V1=VX-VY-omega*(RX+RY);
  int V2=VX+VY+omega*(RX+RY);
  int V3=VX+VY-omega*(RX+RY);
  int V4=VX-VY+omega*(RX+RY);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",6,1500+V1,0); //组合指令
  Serial.println(cmd_return_tmp); //解析ZMotor指令-左电机正向
  delay(10);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",7,1500+(-V2),0); //组合指令
  Serial.println(cmd_return_tmp); //解析ZMotor指令-左电机正向
  delay(10);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",8,1500+V3,0); //组合指令
  Serial.println(cmd_return_tmp); //解析ZMotor指令-左电机正向
  delay(10);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",9,1500+(-V4),0); //组合指令
  Serial.println(cmd_return_tmp); //解析ZMotor指令-左电机正向
  delay(10);
}

//测距函数

void limit(float x,float min,float max)
{
  
if(x<=min){x=min;}
if(x>=max){x=max;}  
}

void pid_init(pid_struct_t *pid,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)//PID初始化函数
{
  pid->kp      = kp;
  pid->ki      = ki;
  pid->kd      = kd;
  pid->i_max   = i_max;
  pid->out_max = out_max;
}
 
 
float pid_calc(pid_struct_t *pid, float ref, float fdb)//PID运算函数
{
  pid->ref = ref;
  pid->fdb = fdb;
  pid->err[1] = pid->err[0];
  pid->err[0] = pid->ref - pid->fdb;
  
  pid->p_out  = pid->kp * pid->err[0];
  pid->i_out += pid->ki * pid->err[0];
  pid->d_out  = pid->kd * (pid->err[0] - pid->err[1]);
  limit(pid->i_out, -pid->i_max, pid->i_max);
  
  pid->output = pid->p_out + pid->i_out + pid->d_out;
  limit(pid->output, -pid->out_max, pid->out_max);
  return pid->output;
}
 

void forward(int speed)
{
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",6,1500+speed,0); //组合指令
  Serial.println(cmd_return_tmp); //解析ZMotor指令-左电机正向
  delay(10);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",7,1500+(-speed),0); //组合指令
  Serial.println(cmd_return_tmp); //解析ZMotor指令-左电机正向
  delay(10);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",8,1500+speed,0); //组合指令
  Serial.println(cmd_return_tmp); //解析ZMotor指令-左电机正向
  delay(10);
  sprintf(cmd_return_tmp, "#%03dP%04dT%04d!",9,1500+(-speed),0); //组合指令
  Serial.println(cmd_return_tmp); //解析ZMotor指令-左电机正向
  delay(10);
}

float checkdistance(void) {  
   //A3为Trig,5为Echo
  digitalWrite(A3, LOW);
  delayMicroseconds(2);
  digitalWrite(A3, HIGH);
  delayMicroseconds(10);
  digitalWrite(A3, LOW);
  float distance = pulseIn(A0, HIGH) / 58.00;//计算距离
  delay(10);
  return distance;//返回值为距离
}
void distance_init(void)
{
  pinMode(A3, OUTPUT);
  pinMode(A0, INPUT);
  pinMode(13, OUTPUT);
  pid_init(&distance_pid, 100, 0, 0, 0, 1000);//P=400,I=0,D=0
}

void start_distance_control(int target_distance)
{
  //检测距离
  int now_distance;
  now_distance=checkdistance();
  Serial.println(now_distance);
  if(now_distance>1000)
  {
    now_distance=1;
  }
  //控制距离
  pid_calc(&distance_pid, target_distance, now_distance);
  forward(-distance_pid.output);
}

// void uart_recieve_init(void)
// {
//   SerialPort.begin(300); 
// }

// void serial_test(void) {
//   if (SerialPort.available()) {
//     uart_what = SerialPort.read(); // 读取接收到的数据
//     Serial.println(uart_recieve);
//   }
// }

// void uart_recieve(void){
//     static size_t index = 0;

//   if (SerialPort.available()) { // 如果串口有数据可用
//     uint8_t byte = SerialPort.read(); // 读取一个字节
//     Serial.print("Received byte: 0x");
//     Serial.println(byte, HEX); // 打印接收到的字节
    
//     /************************************************************等待头帧************************************************ */
//     if (index == 0 && byte != 0x5A) {
//       Serial.println("Waiting for start byte (0x5A)..."); // 打印等待开始字节消息
//       return; // 如果索引为0且字节不是头帧标志0xAA，则忽略这个字节，继续等待头帧
//     }
    
//     packet[index++] = byte; // 将字节存入数据包中，并递增索引
//     /*********************************************************************************** */

//     /************************************************************打印数据包************************************************ */
//     if (index == PACKET_SIZE) { // 如果数据包收集完毕
//       Serial.println("Packet received completely:"); // 打印数据包接收完成消息
//       for (size_t i = 0; i < PACKET_SIZE; ++i) {
//         Serial.print(packet[i], HEX); // 打印数据包中每个字节的十六进制表示
//         Serial.print(" ");
//       }
//       Serial.println();

//       if (validatePacket(packet, PACKET_SIZE)) { // 校验数据包是否有效
//         Serial.println("Valid packet."); // 打印数据包有效消息
//         processPacket(packet, PACKET_SIZE); // 处理接收到的数据包
//       } else {
//         Serial.println("Invalid packet."); // 打印无效数据包消息
//       }

//       index = 0; // 重置索引，准备接收下一个数据包
//     }
//   }
// }

void uart_recieve_2(void){
    static size_t index = 0;

  if (Serial.available()) { // 如果串口有数据可用
    uint8_t byte = Serial.read(); // 读取一个字节
    Serial.print("Received byte: 0x");
    Serial.println(byte, HEX); // 打印接收到的字节
    
    /************************************************************等待头帧************************************************ */
    if (index == 0 && byte != 0x5A) {
      Serial.println("Waiting for start byte (0x5A)..."); // 打印等待开始字节消息
      return; // 如果索引为0且字节不是头帧标志0xAA，则忽略这个字节，继续等待头帧
    }
    
    packet[index++] = byte; // 将字节存入数据包中，并递增索引
    /*********************************************************************************** */

    /************************************************************打印数据包************************************************ */
    if (index == PACKET_SIZE) { // 如果数据包收集完毕
      Serial.println("Packet received completely:"); // 打印数据包接收完成消息
      for (size_t i = 0; i < PACKET_SIZE; ++i) {
        Serial.print(packet[i], HEX); // 打印数据包中每个字节的十六进制表示
        Serial.print(" ");
      }
      Serial.println();

      if (validatePacket(packet, PACKET_SIZE)) { // 校验数据包是否有效
        Serial.println("Valid packet."); // 打印数据包有效消息
        processPacket(packet, PACKET_SIZE); // 处理接收到的数据包
      } else {
        Serial.println("Invalid packet."); // 打印无效数据包消息
      }

      index = 0; // 重置索引，准备接收下一个数据包
    }
  }
}

// 处理接收到的数据包
void processPacket(uint8_t *data, size_t len) {
  // now_delta=packet[4];
  // now_angle=packet[12];
  // if(packet[8]==1)
  // {
  //   now_delta=now_delta;
  // }
  // if(packet[8]==0)
  // {
  //   now_delta=-now_delta;
  // }
  // if(packet[16]==1)
  // {
  //   now_angle=now_angle;
  // }
  // if(packet[16]==0)
  // {
  //   now_angle=-now_angle;
  // }
  color_possess=packet[4];
  level_erro=packet[8];
  level_float=packet[12];
  if(level_float==1)
  {
    level_erro=level_erro;
  }
  if(level_float==0)
  {
    level_erro=-level_erro;
  }
  
  Serial.println("Processing packet:"); // 打印处理数据包消息
  for (size_t i = 0; i < len; ++i) {
    Serial.print(data[i], HEX); // 打印数据包中每个字节的十六进制表示
    Serial.print(" ");
  }
  Serial.println();
}
// 校验数据包
bool validatePacket(uint8_t *data, size_t len) {
  if (data[0] != 0x5A ||data[len-1] != 0xFE ) {
    return false; // 头帧或校验和校验失败，返回 false
  }
  return true; // 数据包有效，返回 true
}

void control_distance_to_wall_and_control(int count,int color_input) {
    // const float target_distance = 35.0f; // 目标距离
    // const float tolerance = 2.0f;        // 容差范围
    
    int color_judge=count%4;

    if (color_judge==color_input)
    {
      while (true) {
          // 获取当前距离
          float current_distance = checkdistance();

          // 判断当前距离与目标距离的差值是否在容差范围内
          if (current_distance >= MIDDLE_DISTANCE - DISTANCE_TOLERANCE && current_distance <= MIDDLE_DISTANCE + DISTANCE_TOLERANCE) {
              printf("距离已调整到目标范围内: %.2f cm\n", current_distance);
              break; // 如果满足条件，退出循环
          }

          // 如果未达到目标范围，调用控制函数进行微调
          start_distance_control(MIDDLE_DISTANCE);
      }

      put_block();//放置物块
    }

    if (count == 1||count==5||count==9) {  // 检查变量 count 是否为 1

      
      while (flag==0) {
        
            // 获取当前距离
            float current_distance = checkdistance();

            // 如果未达到目标范围，调用控制函数进行微调
            start_distance_control(MIDDLE_DISTANCE);
            // 判断当前距离与目标距离的差值是否在容差范围内

            if (current_distance >= MIDDLE_DISTANCE - DISTANCE_TOLERANCE && current_distance <= MIDDLE_DISTANCE + DISTANCE_TOLERANCE) {
                printf("距离已调整到目标范围内: %.2f cm\n", current_distance);
                flag++;
                forward(0);
                break; // 如果满足条件，退出循环
            }
            
            

            
      }
      // color_recognition_cmd_start();
      // get_block();//取走物块
      // delay(1000);
      // color_recognition_cmd_end();
      flag=0;
    } else {
        printf("count 不为 1，未执行距离控制逻辑。\n");
    }
}

// void uart_send(uint8_t* dataArray, size_t length) {
//     for (size_t i = 0; i < length; i++) {
//         SerialPort.write(dataArray[i]);  // 发送数组中的每个字节
//     }
// }

void put_block(void)//放置物块
{
  control_arm(0,-135,90,45,0,0);
  control_arm(0,-135,90,45,0,0);
}

void get_block(void)//拿走物块
{
  control_arm(0,-135,90,45,0,0);
  control_arm(0,-135,90,45,0,0);
}

void draw_ten(void)//画十
{
  draw_stroke(300,0,260,300,0,150);
  controlServo(2,-135);
  delay(1000);
  draw_stroke(300,-60,200,300,60,200);
}
void draw_two(void)//画二
{
  draw_stroke(300,-60,200,300,60,200);
  controlServo(2,-135);
  delay(1000);
  draw_stroke(300,-60,110,300,60,110);
}
void draw_T(void)//画T
{
  draw_stroke(300,0,180,300,0,120);
  controlServo(2,-135);
  delay(1000);
  draw_stroke(300,-60,200,300,60,200);
}

void color_recognition_cmd_start(void)
{
  //向openmv发送开始识别颜色的指令
  dataToSend[0]=0x5A;
  dataToSend[1]=1;
  dataToSend[SEND_SIZE-1]=0xFE;
  uart_send(dataToSend, SEND_SIZE);
}

void color_recognition_cmd_end(void)
{
  //向openmv发送停止识别颜色的指令
  dataToSend[0]=0x5A;
  dataToSend[1]=0;
  dataToSend[SEND_SIZE-1]=0xFE;
  uart_send(dataToSend, SEND_SIZE);
}

void get_actual_distance(void)
{
  //A3为Trig,5为Echo
  digitalWrite(A3, LOW);
  delayMicroseconds(2);
  digitalWrite(A3, HIGH);
  delayMicroseconds(10);
  digitalWrite(A3, LOW);
  actual_distance = pulseIn(A0, HIGH) / 58.00;//计算距离
  delay(10);
  
}

void stop_and_get(void)
{
  // digitalWrite(2,0);
  //停止
  set_openmv_state(DO_NOTHING);//告诉openmv停止

  // set_openmv_state(2);//识别颜色

  forward(BACK_VELOCITY);
  delay(BACKTIME);//后退 我不好说 依据需要
  forward(0);

  have_a_look();//机械臂观察颜色姿态
  delay(2000);//等待机械臂稳定下来

  set_openmv_state(COLOR_MODE);//开启颜色识别矫正
  delay(6000);//等待上位机矫正底盘位置
  color_digital_detect();//收集颜色信息 迭代ball_color
  delay(1000);

  set_openmv_state(DO_NOTHING);//告诉openmv停止 别抽风

  delay(2000);
  catch_it();//机械臂抓取
  arm_normal_type();//机械臂复位
  delay(1000);

  set_openmv_state(TRACK_MODE);//开始寻迹

  

  // //接收串口信息 颜色 偏移量 
  // uart_receive_3();
  // delay(500);//暂停半秒 再次接收 确保稳定
  // uart_receive_3();
  // ball_color=color_possess;
  // //下面这一段要改
  // forward(300);
  // delay(BACKTIME);
  // forward(0);
   //拿走物块
  

  // arm_get();
  // arm_normal_type();
  // delay(1000);
  // digitalWrite(2,1);

   

}

void stop_and_put(void)
{
  // digitalWrite(2,0);
  // //停止
  // forward(-300);
  // delay(BACKTIME);
  // forward(0);
  // //放置物块
  // arm_put();
  // arm_normal_type();
  // delay(1000);
  // digitalWrite(2,1);

  set_openmv_state(DO_NOTHING);//告诉openmv停止
  
  // Magnam_forward(-200, 0, -2);//这个可能需要上位机完成 我不好说 先保留
  // delay(1000);//后退 我不好说
  // forward(0);//停止 我不好说
  delay(2000);
  have_a_look();

  delay(2000);
  set_openmv_state(BLACK_MODE);

  delay(6000);//等待openmv调整位置
  
  set_openmv_state(DO_NOTHING);//告诉openmv停止
  delay(2000);
  arm_put();//随意抛弃你的球
  delay(POSTURE_DELAY);
  arm_normal_type();//复位机械臂位置
  delay(POSTURE_DELAY);

  set_openmv_state(TRACK_MODE);//告诉openmv开始寻迹

}

void time_count(void)
{
  // get_actual_distance();
  actual_distance=checkdistance();
  Serial.println("distance:");
  Serial.print(actual_distance);
  curren_time=millis();

  if((curren_time-last_time)>=COOL_DOWN_TIME)
  {
    if(actual_distance<=DETECT_DISTANCE)
    {
      new_count++;
      last_time=millis();
    }
    if(new_count%4==1)
    {
      stop_and_get();
      last_time=millis();
    }
    color_middle=new_count%4;
    if(color_middle==color_recieve)
    {
      stop_and_put();
      last_time=millis();
    }
  }
  Serial.println("count:");
  Serial.print(new_count);

}

// void display_oled(void)
// {
//   u8g2.clearBuffer();          // clear the internal memory
//   u8g2.setFont(u8g2_font_ncenB14_tr);  // 这里可以修改字体大小
//   u8g2.setCursor(0,32);
//   u8g2.print(new_count);
//   u8g2.sendBuffer();         // transfer internal memory to the display
//   delay(10);
// }
void arm_normal_type(void)
{
  control_arm(0,-80,48,125,5,110);//常态闭合
  delay(POSTURE_DELAY);
}
void arm_get(void)
{
  control_arm(0,-45,45,0,5,0);//张开
  delay(POSTURE_DELAY);
  // control_arm(-105,-45,45,0,5,0);
  control_arm(-105,-70,90,40,5,0);//have a look
  delay(POSTURE_DELAY);
  control_arm(-105,-15,45,0,5,110);//闭合
  delay(POSTURE_DELAY);
  control_arm(-105,-45,45,0,5,110);
  delay(POSTURE_DELAY);
}

void arm_put(void)
{
  // control_arm(0,-45,45,0,5,110);//闭合
  delay(POSTURE_DELAY);
  control_arm(-105,-45,45,0,5,110);
  delay(POSTURE_DELAY);
  // control_arm(-105,-15,45,0,5,0);//张开
  control_arm(-105,-30,45,0,5,110);//张开
  delay(POSTURE_DELAY);
  control_arm(-105,-30,45,0,5,0);//张开
  delay(POSTURE_DELAY);
  control_arm(-105,-45,45,0,5,0);
  delay(POSTURE_DELAY);
}

//写字任务的模拟函数
void stop_and_write(int color)
{
  set_openmv_state(DO_NOTHING);

  forward(0);

  Magnam_forward(-200, 0, 1);//后退
  delay(1500);

  Magnam_forward(0, 0, 13.15);//旋转

  delay(1700);

  Magnam_forward(0, 0, 0);//停止

  control_arm(0, -135, 45 , 0 , 5, 0);//抬起机械臂

  delay(1000);

  Magnam_forward(400,0,1.5);//怼墙

  delay(3500);

  Magnam_forward(0,0,0);

  Magnam_forward(-200,0,0);//后退

  delay(1800);

  Magnam_forward(0,0,0);

  control_arm(0, -135, 45 , 0 , 5, 0);
  delay(1000);

  // 根据颜色决定写字内容
  switch (color)
  {
  case 2:
    Serial.println("Writing: Color 2 Detected!");
    draw_T();
    break;
  case 3:
    Serial.println("Writing: Color 3 Detected!");
    draw_two();
    break;
  case 4:
    Serial.println("Writing: Color 4 Detected!");
    draw_ten();
    break;
  default:
    Serial.println("Writing: Unknown Color!");
    break;
  }
}

// void time_count_2(void)
// {
//   actual_distance = checkdistance(); // 每次检测到障碍物
//   Serial.println("distance:");
//   Serial.print(actual_distance);
//   curren_time = millis();

//   // 冷却时间判断
//   if ((curren_time - last_time) >= COOL_DOWN_TIME)
//   {
//     if (actual_distance <= DETECT_DISTANCE)
//     {
//       if (!is_writing)
//       {
//         // 更新障碍物编号
//         obstacle_count++;
//         if (obstacle_count > 4)
//         {
//           obstacle_count = 1; // 超过4个障碍物，循环重置
//           task_count++;       // 每次经过4个障碍物后，任务计数增加
//         }

//         // 判断任务计数
//         if (task_count == 3)
//         {
//           Serial.println("All tasks completed! Entering writing task...");
//           is_writing = true; // 开始写字任务
//           obstacle_count = 0; // 重置障碍物计数
//           return;
//         }

//         // 判断当前障碍物
//         switch (obstacle_count)
//         {
//         case 1:
//           // 第1号障碍物：取球
//           if (!has_ball) // 只有没有持球时才抓取
//           {
//             ball_color = 3; // 获取球的颜色信息
//             stop_and_get();                // 停车并抓取
//             has_ball = true;               // 更新持球状态
//             Serial.print("Picked up ball with color: ");
//             Serial.println(ball_color);
//           }
//           break;

//         case 2:
//         case 3:
//         case 4:
//           // 2号、3号、4号障碍物：放置球
//           if (has_ball && ball_color == obstacle_count)
//           {
//             stop_and_put(); // 停车并放置
//             has_ball = false;
//             Serial.print("Placed ball at obstacle: ");
//             Serial.println(obstacle_count);
//           }
//           break;
//         }
//       }
//       else
//       {

//         // 在障碍物处写字
//         stop_and_write(ball_color); // 根据最后一个小球颜色决定写字内容
//         Serial.print("Writing task at obstacle ");
//         Serial.print(obstacle_count);
//         Serial.print(" with color: ");
//         Serial.println(ball_color);
//       }

//       // 更新最后时间戳
//       last_time = millis();
//     }
//   }

//   Serial.print("Current obstacle: ");
//   Serial.println(obstacle_count);
//   Serial.print("Current task count: ");
//   Serial.println(task_count);
// }

// 获取中值
float median_filter(float new_distance) {
    // 插入新值
    for (int i = MEDIAN_SIZE - 1; i > 0; i--) {
        median_window[i] = median_window[i - 1];
    }
    median_window[0] = new_distance;

    // 复制并排序
    int sorted[MEDIAN_SIZE];
    memcpy(sorted, median_window, sizeof(median_window));
    for (int i = 0; i < MEDIAN_SIZE - 1; i++) {
        for (int j = 0; j < MEDIAN_SIZE - 1 - i; j++) {
            if (sorted[j] > sorted[j + 1]) {
                int temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    // 返回中间值
    return sorted[MEDIAN_SIZE / 2];
}

void time_count_2(void)
{
  // actual_distance = checkdistance(); // 每次检测到障碍物
  actual_distance = median_filter(checkdistance()); // 使用滤波后的距离值
  Serial.print("Distance detected: ");
  Serial.println(actual_distance);
  curren_time = millis();

  // 冷却时间判断
  if ((curren_time - last_time) >= COOL_DOWN_TIME)
  {
    Serial.println("Cooldown period passed, checking for obstacles...");
    if (actual_distance <= DETECT_DISTANCE)
    {
      Serial.println("Obstacle detected within range!");

      if (!is_writing)
      {
        // 更新障碍物编号
        obstacle_count++;
        Serial.print("Updated obstacle count: ");
        Serial.println(obstacle_count);

        if (obstacle_count > 4)
        {
          obstacle_count = 1; // 超过4个障碍物，循环重置
          task_count++;       // 每次经过4个障碍物后，任务计数增加
          Serial.print("Task count incremented: ");
          Serial.println(task_count);
        }

        // 判断任务计数
        if (task_count == 3)
        {
          Serial.println("All tasks completed! Entering writing task...");
          is_writing = true;  // 开始写字任务
          obstacle_count = 0; // 重置障碍物计数
          return;
        }

        // 判断当前障碍物
        switch (obstacle_count)
        {
        case 1:
          // 第1号障碍物：取球
          Serial.println("At obstacle 1: Attempting to pick up ball...");
          if (!has_ball) // 只有没有持球时才抓取
          {
            //ball_color = 3;这一段我写在下面这个函数里了 // 获取球的颜色信息************************************************************要改
            stop_and_get();                 // 停车并抓取 ball_color迭代
            has_ball = true;                // 更新持球状态
            Serial.print("Picked up ball with color: ");
            Serial.println(ball_color);
          }
          else
          {
            Serial.println("Already holding a ball, skipping pickup.");
          }
          break;

        case 2:
        case 3:
        case 4:
          // 2号、3号、4号障碍物：放置球
          Serial.print("At obstacle ");
          Serial.print(obstacle_count);
          Serial.println(": Attempting to place ball...");
          if (has_ball && ball_color == obstacle_count)
          {
            stop_and_put(); // 停车并放置
            has_ball = false;
            Serial.print("Placed ball at obstacle: ");
            Serial.println(obstacle_count);
          }
          else
          {
            Serial.println("No ball to place or color mismatch.");
          }
          break;
        }
      }
      else
      {
        // 写字任务逻辑
        Serial.println("In writing task mode...");
        obstacle_count++;
        Serial.print("Writing at obstacle count: ");
        Serial.println(obstacle_count);

        if (obstacle_count > 4)
        {
          obstacle_count = 1; // 超过4个障碍物，循环重置
          Serial.println("Reset obstacle count for writing task.");
        }

        // 在障碍物处写字
        stop_and_write(ball_color); // 根据最后一个小球颜色决定写字内容
        Serial.print("Writing task at obstacle ");
        Serial.print(obstacle_count);
        Serial.print(" with color: ");
        Serial.println(ball_color);
      }

      // 更新最后时间戳
      last_time = millis();
      Serial.print("Updated last_time: ");
      Serial.println(last_time);
    }
    else
    {
      Serial.println("No obstacle within detect distance.");
    }
  }
  else
  {
    Serial.println("Cooldown period not yet passed.");
  }

  Serial.print("Current obstacle: ");
  Serial.println(obstacle_count);
  Serial.print("Current task count: ");
  Serial.println(task_count);
  Serial.print("Is writing: ");
  Serial.println(is_writing ? "true" : "false");
  Serial.print("Has ball: ");
  Serial.println(has_ball ? "true" : "false");
}

//****************************************************/
void W_control(void)
{
  pid_calc(&W_pid, 0, now_angle);
  W=W_pid.output;
}

void VY_control(void)
{
  pid_calc(&VY_pid, 0, now_delta);
  Vy=VY_pid.output;
}

void integrity_control(void)
{
  Vx=300;
  W_control();
  VY_control();
  Magnam_forward(Vx,Vy,W);
}

void the_whole_task(void)
{
  uart_recieve_2();
  integrity_control();
  time_count_2();

}

void volocity_PID_init(void)
{
  pid_init(&W_pid, 1, 0, 0, 0, 300);//1
  pid_init(&VY_pid,15, 0, 0, 0, 600);

}

void uart_receive_3(void) {
    size_t index = 0; // 数据包索引

    while (true) { // 持续读取数据直到完整包接收完成
        if (Serial.available()) { // 如果串口有数据可用
            uint8_t byte = Serial.read(); // 读取一个字节
            Serial.print("Received byte: 0x");
            Serial.println(byte, HEX); // 打印接收到的字节

            // 等待头帧
            if (index == 0 && byte != 0x5A) {
                Serial.println("Waiting for start byte (0x5A)..."); // 打印等待开始字节消息
                continue; // 如果不是头帧，继续等待
            }

            // 存储字节到数据包
            packet[index++] = byte;

            // 数据包接收完成
            if (index == PACKET_SIZE) {
                Serial.println("Packet received completely:"); // 打印数据包接收完成消息
                for (size_t i = 0; i < PACKET_SIZE; ++i) {
                    Serial.print(packet[i], HEX); // 打印数据包中每个字节的十六进制表示
                    Serial.print(" ");
                }
                Serial.println();

                // 校验数据包
                if (validatePacket(packet, PACKET_SIZE)) {
                    Serial.println("Valid packet."); // 打印数据包有效消息
                    processPacket(packet, PACKET_SIZE); // 处理接收到的数据包
                } else {
                    Serial.println("Invalid packet."); // 打印无效数据包消息
                }

                break; // 跳出循环，结束函数
            }
        }
    }
}
void state_change(void)
{
  // 读取 A5 和 A4 的状态
  int stateA5 = digitalRead(A5);
  int stateA4 = digitalRead(A4);
  
  if (stateA5 == 0 && stateA4 == 0) // 第一圈之前
  {
    task_count = 0;
    obstacle_count = 0;
    waitFor40Seconds = true; // 设置标志，需要等待40秒
    Serial.println("State: 第一圈之前, task_count = 0");
  }
  else if (stateA5 == 1 && stateA4 == 1) // 第三圈之前
  {
    task_count = 1;
    obstacle_count = 4;
    waitFor40Seconds = false; // 不需要等待40秒
    Serial.println("State: 第三圈之前, task_count = 3");
  }
  else // 第二圈之前
  {
    task_count = 0;
    obstacle_count = 4;
    waitFor40Seconds = false; // 不需要等待40秒
    Serial.println("State: 第二圈之前, task_count = 3");
  }
}
void color_digital_detect(void)
{
  // 读取 A2 和 A1 的状态
  int state1 = digitalRead(A2);
  int state2 = digitalRead(A1);

  if (state1 == 0 && state2 == 0) // 蓝色
  {
    ball_color=2;
    Serial.println("Color: 蓝色");
  }
  else if (state1 == 0 && state2 == 1) // 绿色
  {
    ball_color=3;
    Serial.println("Color: 绿色");
  }
  else if (state1 == 1 && state2 == 1) // 红色
  {
    ball_color=4;
    Serial.println("Color: 红色");
  }
  else // 无效状态
  {
    Serial.println("Color: 未知状态");
  }
  Serial.print(ball_color);
}

void set_openmv_state(int mode)
{
  if(mode==1)//啥也不干
  {
    digitalWrite(2,0);
    digitalWrite(11,0);
    forward(0);
    forward(0); //反复执行 确保无误 能停下来  
    forward(0);
    forward(0); //反复执行 确保无误 能停下来  
    forward(0); 

  }
  else if(mode==2)//颜色识别
  {
    digitalWrite(2,1);
    digitalWrite(11,0);
  }
  else if(mode==3)//寻迹
  {
    digitalWrite(2,0);
    digitalWrite(11,1); 
  }
  else if(mode==4)
  {
    digitalWrite(2,1);
    digitalWrite(11,1);    
  }
  else{
    digitalWrite(2,0);
    digitalWrite(11,0);
  }

}

void have_a_look(void)
{
  control_arm(0,-45,45,0,5,110);//闭合
  delay(POSTURE_DELAY);
  // control_arm(-105,-45,45,0,5,0);
  control_arm(-105,-70,90,40,5,110);//have a look
  delay(POSTURE_DELAY);

}

void catch_it(void)
{
  // control_arm(-105,-15,45,0,5,110);//闭合
  // delay(POSTURE_DELAY);
  // control_arm(-105,-45,45,0,5,110);
  // delay(POSTURE_DELAY);
  control_arm(-105,-70,55,-45,5,0);//上举找到合适位置
  delay(POSTURE_DELAY);

//******************************************************************************
//此处添加新的东西  
  control_arm(-105,-40,55,-45,5,0);//下放
  delay(POSTURE_DELAY);
 
  control_arm(-105,-20,55,-45,5,0);//下放
  delay(POSTURE_DELAY);  

  control_arm(-105,-20,65,-55,5,0);//下放
  delay(POSTURE_DELAY);   

  control_arm(-105,-8,70,-55,5,0);//下放
  delay(POSTURE_DELAY); 

//******************************************************************************  
  control_arm(-105,-5,55,-45,5,0);//下放 终点
  delay(POSTURE_DELAY);
  control_arm(-105,-4,55,-45,5,130);//闭合

  // control_arm(-105,-15,50,0,5,110);//闭合

  delay(POSTURE_DELAY);
  control_arm(-105,-45,45,0,5,130);
  delay(POSTURE_DELAY);
}
void change_speed(void)
{
  now_time=millis();
  if(now_time>0&&now_time<=FIRST_SPEED_SWITCH)
  {
    digitalWrite(4,LOW);
  }
  if(now_time>FIRST_SPEED_SWITCH&&now_time<=SECOND_SPEED_SWITCH)
  {
    digitalWrite(4,HIGH);
  }
  if(now_time>SECOND_SPEED_SWITCH)
  {

    digitalWrite(4,LOW);
  }
}