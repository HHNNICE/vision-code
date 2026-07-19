/************************************************************************************************
 * 版本号：V2.1 - 摄像头与机械臂同侧配置
 * 功能：6自由度机械臂 + OpenMV视觉识别 + 同侧/斜拍标定
 ************************************************************************************************/

#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// ==================== 摄像头安装配置 ====================

// 摄像头安装位置选择（取消注释对应配置）
#define CAM_POS_OVERHEAD    // 垂直俯视（默认）

// 摄像头方向补偿（如果图像需要镜像）
#define CAM_MIRROR_X        false   // 水平镜像
#define CAM_MIRROR_Y        true   // 垂直镜像

// ==================== 标定配置区 ====================

// 图像参数
#define IMG_WIDTH       320
#define IMG_HEIGHT      240
#define CENTER_X        160
#define CENTER_Y        120

#ifdef CAM_POS_OVERHEAD
  // 垂直俯视配置
  #define CAL_CENTER_X    0       // 图像中心X
  #define CAL_CENTER_Y    23   // 图像中心Y（距离）
  #define CAL_BOTTOM_Y    35      // 图像底部Y（更远）
  #define SCALE_X         0.10    // X方向比例
  #define SCALE_Y         ((CAL_BOTTOM_Y - CAL_CENTER_Y) / (200.0 - CENTER_Y))
  #define TILT_COMPENSATION 0     // 无倾斜补偿
  

  
#endif

// 抓取参数
#define GRAB_HEIGHT     4.5     // 物体表面高度

// ==================== 机械臂几何参数 ====================

float R = 6.5;          // 底座半径
float length_1 = 6;   // 底座高度
float length_2 = 12.0;  // 大臂
float length_3 = 9.0;   // 小臂
float length_4 = 7.0;   // 手腕

// ==================== 舵机和全局变量（保持不变）====================

Servo servo1, servo2, servo3, servo4, servo5, servo6;
#define SERVO1_PIN 9
#define SERVO2_PIN 10
#define SERVO3_PIN 11
#define SERVO4_PIN 12
#define SERVO5_PIN 3
#define SERVO6_PIN 4

double now_angle_1 = 0, now_angle_2 = 0, now_angle_3 = 0, now_angle_4 = 0, now_angle_5 = 0, now_angle_6 = 20;
double target_angle_1 = 0, target_angle_2 = 0, target_angle_3 = 0, target_angle_4 = 0, target_angle_5 = 0, target_angle_6 = 0;

char colorType = 'A';
int targetX = 0, targetY = 0;
bool targetDetected = false, grabTriggered = false;

#define STATE_IDLE 0
#define STATE_GRAB 1
#define STATE_LIFT 2
#define STATE_TRANSFER 3
#define STATE_RELEASE 4
#define STATE_RESET 5

int grabState = STATE_IDLE;
bool isProcessing = false;

#define MAX_BUFFER 16
#define FRAME_LEN 8       // 帧长度: 2头 + 1颜色 + 2(X) + 2(Y) + 1尾 = 8字节
unsigned char rxBuffer[MAX_BUFFER];
int rxIndex = 0;

// ==================== 函数声明 ====================

void pwm_start();
void pwm_out(double a1, double a2, double a3, double a4, double a5, double a6);
void servo_angle_calculate(float x, float y, float z);
void servo_control(double t1, double t2, double t3, double t4, double t5, double t6);
void servo_reset_begin();
void processSerial();
void visionControl();
void updateServos();
void pixelToArm(float px, float py, float &ax, float &ay);
void resetToIdle();
void sendReady();
void transfer_blue();
void transfer_white();
void transfer_red();

// ==================== 初始化 ====================

void setup() {
    Serial.begin(9600);
    delay(500);
    
    Serial.println("\n========================================");
    Serial.println("6DOF Robot Arm - Vision Grasp System");
    
    #ifdef CAM_POS_OVERHEAD
      Serial.println("Mode: OVERHEAD (垂直俯视)");
    
    #endif
    
    Serial.print("Calibration: Center(");
    Serial.print(CAL_CENTER_X);
    Serial.print(",");
    Serial.print(CAL_CENTER_Y);
    #ifdef CAM_POS_OVERHEAD
      Serial.print(") ScaleY=");
      Serial.println(SCALE_Y, 4);
    #else
      Serial.print(") ScaleX=");
      Serial.print(SCALE_X);
      Serial.print(" ScaleY=");
      Serial.println(SCALE_Y);
    #endif
    
    pwm_start();
    servo_reset_begin();
    
    Serial.println("[INIT] Ready");
    Serial.println("========================================\n");
    
    sendReady();
}

void loop() {
    if (grabState == STATE_IDLE && !isProcessing) {
        processSerial();
    }
    
    if (grabTriggered && targetDetected && grabState == STATE_IDLE) {
        visionControl();
    }
    
    if (grabState != STATE_IDLE) {
        updateServos();
    }
    
    delay(20);
}

// ==================== 舵机控制（保持不变）====================

void pwm_start() {
    servo1.attach(SERVO1_PIN);
    servo2.attach(SERVO2_PIN);
    servo3.attach(SERVO3_PIN);
    servo4.attach(SERVO4_PIN);
    servo5.attach(SERVO5_PIN);
    servo6.attach(SERVO6_PIN);
}

void pwm_out(double a1, double a2, double a3, double a4, double a5, double a6) {
    a1 = constrain(a1, -90, 90);
    a2 = constrain(a2, -90, 90);
    a3 = constrain(a3, -90, 90);
    a4 = constrain(a4, -90, 90);
    a5 = constrain(a5, -90, 90);
    a6 = constrain(a6, -90, 90);
    
    int p1 = int(a1 + 90);
    int p2 = int(a2 + 90);
    int p3 = int(-a3 + 90);
    int p4 = int(a4 + 90);
    int p5 = int(a5 * 65 / 90 + 90);
    int p6 = int(a6 + 90);
    
    p1 = constrain(p1, 0, 180);
    p2 = constrain(p2, 0, 180);
    p3 = constrain(p3, 0, 180);
    p4 = constrain(p4, 0, 180);
    p5 = constrain(p5, 0, 180);
    p6 = constrain(p6, 0, 180);
    
    servo1.write(p1);
    servo2.write(p2);
    servo3.write(p3);
    servo4.write(p4);
    servo5.write(p5);
    servo6.write(p6);
}

void servo_control(double t1, double t2, double t3, double t4, double t5, double t6) {
    double step = 3.0;
    int delay_ms = 30;
    
    // 底座
    while (abs(t1 - now_angle_1) > step) {
        now_angle_1 += (t1 > now_angle_1) ? step : -step;
        pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
        delay(delay_ms);
    }
    now_angle_1 = t1;
    pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
    
    // 大臂
    while (abs(t2 - now_angle_2) > step) {
        now_angle_2 += (t2 > now_angle_2) ? step : -step;
        pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
        delay(delay_ms);
    }
    now_angle_2 = t2;
    pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
    
    // 小臂
    while (abs(t3 - now_angle_3) > step) {
        now_angle_3 += (t3 > now_angle_3) ? step : -step;
        pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
        delay(delay_ms);
    }
    now_angle_3 = t3;
    pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
    
    // 手腕旋转
    while (abs(t4 - now_angle_4) > step) {
        now_angle_4 += (t4 > now_angle_4) ? step : -step;
        pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
        delay(delay_ms);
    }
    now_angle_4 = t4;
    pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
    
    // 手腕俯仰
    while (abs(t5 - now_angle_5) > step) {
        now_angle_5 += (t5 > now_angle_5) ? step : -step;
        pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
        delay(delay_ms);
    }
    now_angle_5 = t5;
    pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
    
    // 抓手
    while (abs(t6 - now_angle_6) > step) {
        now_angle_6 += (t6 > now_angle_6) ? step : -step;
        pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
        delay(delay_ms);
    }
    now_angle_6 = t6;
    pwm_out(now_angle_1, now_angle_2, now_angle_3, now_angle_4, now_angle_5, now_angle_6);
    
    delay(500);
}

void servo_reset_begin() {
    Serial.println("[MOVE] Reset to home");
    servo_control(0, 0, 0, 0, 0, 20);
    delay(500);
}

// ==================== 逆运动学====================

void servo_angle_calculate(float target_x, float target_y, float target_z) {
    Serial.print("[IK] Target: (");
    Serial.print(target_x, 2); Serial.print(", ");
    Serial.print(target_y, 2); Serial.print(", ");
    Serial.print(target_z, 2); Serial.println(")");
    
    float reach = sqrt(target_x * target_x + target_y * target_y);
    float max_reach = length_2 + length_3 + length_4;
    
    if (target_y < 3 || target_y > 35) {
        Serial.println("[IK] Y out of range");
        target_angle_1 = target_angle_2 = target_angle_3 = target_angle_5 = 0;
        return;
    }
    
    if (target_z < 2 || target_z > length_1 + max_reach) {
        Serial.println("[IK] Z out of range");
        target_angle_1 = target_angle_2 = target_angle_3 = target_angle_5 = 0;
        return;
    }
    
    if (reach > max_reach) {
        Serial.println("[IK] Reach out of range");
        target_angle_1 = target_angle_2 = target_angle_3 = target_angle_5 = 0;
        return;
    }
    
    float len_1 = length_1, len_2 = length_2, len_3 = length_3, len_4 = length_4;
    float bottom_r = R;
    float j1, j2, j3, j4, L, H, j_sum, len, high;
    float cos_j3, sin_j3, cos_j2, sin_j2, k1, k2;
    int n = 0, m = 0;
    
    // j1
    if (target_x == 0 && target_y == 0) {
        j1 = 90;
    } else {
        j1 = -atan2(target_x, target_y + bottom_r) * 57.29578;
    }
    if (j1 < -90) j1 = -90;
    if (j1 > 90) j1 = 90;
    
    // 第一阶段：计数
    for (int i = 0; i <= 180; i++) {
        j_sum = 3.1415927 * i / 180.0;
        len = sqrt((target_y + bottom_r - 3) * (target_y + bottom_r - 3) + target_x * target_x);
        high = target_z;
        L = len - len_4 * sin(j_sum);
        H = high - len_4 * cos(j_sum) - len_1;
        
        float dist_sq = L * L + H * H;
        float max_arm = len_2 + len_3;
        float min_arm = fabs(len_2 - len_3);
        
        if (dist_sq > max_arm * max_arm || dist_sq < min_arm * min_arm) continue;
        
        cos_j3 = (L * L + H * H - len_2 * len_2 - len_3 * len_3) / (2 * len_2 * len_3);
        if (cos_j3 < -1.0 || cos_j3 > 1.0) continue;
        
        sin_j3 = sqrt(1 - cos_j3 * cos_j3);
        j3 = atan2(sin_j3, cos_j3) * 57.29578;
        
        k2 = len_3 * sin(j3 / 57.29578);
        k1 = len_2 + len_3 * cos(j3 / 57.29578);
        cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
        
        if (cos_j2 < -1.0 || cos_j2 > 1.0) continue;
        
        sin_j2 = sqrt(1 - cos_j2 * cos_j2);
        j2 = atan2(sin_j2, cos_j2) * 57.29578;
        j4 = j_sum * 57.29578 - j2 - j3;
        
        if (j2 > 0 && j3 > 0 && j4 > -90 && j2 < 90 && j3 < 90 && j4 < 90) {
            n++;
        }
    }
    
    Serial.print("[IK] Solutions: "); Serial.println(n);
    
    if (n == 0) {
        Serial.println("[IK] No solution");
        target_angle_1 = target_angle_2 = target_angle_3 = target_angle_5 = 0;
        return;
    }
    
    // 第二阶段：找中间解
    for (int i = 0; i <= 180; i++) {
        j_sum = 3.1415927 * i / 180.0;
        len = sqrt((target_y + bottom_r - 3) * (target_y + bottom_r - 3) + target_x * target_x);
        high = target_z;
        L = len - len_4 * sin(j_sum);
        H = high - len_4 * cos(j_sum) - len_1;
        
        float dist_sq = L * L + H * H;
        float max_arm = len_2 + len_3;
        float min_arm = fabs(len_2 - len_3);
        
        if (dist_sq > max_arm * max_arm || dist_sq < min_arm * min_arm) continue;
        
        cos_j3 = (L * L + H * H - len_2 * len_2 - len_3 * len_3) / (2 * len_2 * len_3);
        if (cos_j3 < -1.0 || cos_j3 > 1.0) continue;
        
        sin_j3 = sqrt(1 - cos_j3 * cos_j3);
        j3 = atan2(sin_j3, cos_j3) * 57.29578;
        
        k2 = len_3 * sin(j3 / 57.29578);
        k1 = len_2 + len_3 * cos(j3 / 57.29578);
        cos_j2 = (k2 * L + k1 * H) / (k1 * k1 + k2 * k2);
        
        if (cos_j2 < -1.0 || cos_j2 > 1.0) continue;
        
        sin_j2 = sqrt(1 - cos_j2 * cos_j2);
        j2 = atan2(sin_j2, cos_j2) * 57.29578;
        j4 = j_sum * 57.29578 - j2 - j3;
        
        if (j2 > 0 && j3 > 0 && j4 > -90 && j2 < 90 && j3 < 90 && j4 < 90) {
            m++;
            if (m == n / 2 || m == (n + 1) / 2) break;
        }
    }
    
    target_angle_1 = j1 ;
    target_angle_2 = j2;
    target_angle_3 = j3;
    target_angle_5 = j4;
    
    Serial.print("[IK] RESULT: J1="); Serial.print(target_angle_1, 2);
    Serial.print(" J2="); Serial.print(target_angle_2, 2);
    Serial.print(" J3="); Serial.print(target_angle_3, 2);
    Serial.print(" J5="); Serial.println(target_angle_5, 2);
    Serial.println("[IK] RANGE_OK");
}

// ==================== 串口通信 ====================

void processSerial() {
    while (Serial.available() > 0) {
        unsigned char c = Serial.read();

        if (c == 0x2C && rxIndex == 0) {
            rxBuffer[rxIndex++] = c;
        } else if (rxIndex > 0) {
            rxBuffer[rxIndex++] = c;

            if (rxIndex >= FRAME_LEN) {
                // 验证帧头帧尾: 0x2C 0x12 ... 0x5B
                if (rxBuffer[0] == 0x2C && rxBuffer[1] == 0x12 && rxBuffer[FRAME_LEN - 1] == 0x5B) {
                    char color = rxBuffer[2];
                    // X和Y各占2字节，小端序 (low byte first)
                    int x = rxBuffer[3] | (rxBuffer[4] << 8);
                    int y = rxBuffer[5] | (rxBuffer[6] << 8);

                    // 镜像补偿
                    if (CAM_MIRROR_X) x = IMG_WIDTH - x;
                    if (CAM_MIRROR_Y) y = IMG_HEIGHT - y;

                    if (color == 'B' || color == 'P' || color == 'R') {
                        colorType = color;
                        targetX = x;
                        targetY = y;
                        targetDetected = true;
                        grabTriggered = true;
                        isProcessing = true;

                        Serial.print("[RX] Color="); Serial.print(color);
                        Serial.print(" Pixel("); Serial.print(x);
                        Serial.print(","); Serial.print(y); Serial.println(")");
                    }
                } else {
                    Serial.println("[RX] Bad frame");
                }
                rxIndex = 0;
            }

            if (rxIndex >= MAX_BUFFER) {
                Serial.println("[RX] Buffer overflow, reset");
                rxIndex = 0;
            }
        }
    }
}

void sendReady() {
    Serial.println("[READY]");
}

// ==================== 坐标转换====================

void pixelToArm(float px, float py, float &ax, float &ay) {
    float dx, dy;
    
    #ifdef CAM_POS_OVERHEAD
        // 垂直俯视：直接线性映射
        dx = px - CENTER_X;
        dy = CENTER_Y - py;  // 图像Y向下，机械臂Y向前，需要反向
        
        ax = CAL_CENTER_X + dx * SCALE_X;
        ay = CAL_CENTER_Y + dy * SCALE_Y - 3;
        
    
    #endif
    
    // 安全检查
    if (ax < -20 || ax > 20 || ay < 3 || ay > 35) {
        Serial.println("[CONV] Coordinate out of safe range");
    }
    
    ax = constrain(ax, -20, 20);
    ay = constrain(ay, 3, 35);
    
    Serial.print("[CONV] Pixel("); Serial.print(px, 0);
    Serial.print(","); Serial.print(py, 0);
    Serial.print(") -> Arm("); Serial.print(ax, 1);
    Serial.print(","); Serial.print(ay, 1); Serial.println(")");
}

// ==================== 视觉控制 ====================

void visionControl() {
    if (!grabTriggered || !targetDetected || grabState != STATE_IDLE) return;
    
    float arm_x, arm_y;
    float arm_z = GRAB_HEIGHT;
    
    pixelToArm((float)targetX, (float)targetY, arm_x, arm_y);
    servo_angle_calculate(arm_x, arm_y, arm_z);
    
    if (target_angle_2 == 0 && target_angle_3 == 0) {
        Serial.println("[ERROR] IK failed");
        resetToIdle();
        return;
    }
    
    grabState = STATE_GRAB;
    Serial.println("[STATE] Start GRAB");
}

void resetToIdle() {
    targetDetected = false;
    grabTriggered = false;
    isProcessing = false;
    grabState = STATE_IDLE;
    sendReady();
    Serial.println("[STATE] Reset to IDLE");
}

// ==================== 状态机 ====================

void updateServos() {
    switch (grabState) {
        case STATE_GRAB:
            Serial.println("[ACTION] Approach");
            servo_control(target_angle_1, target_angle_2, target_angle_3, 
                         target_angle_4, target_angle_5, 50);
            delay(500);
            
            Serial.println("[ACTION] Grasp");
            servo_control(target_angle_1, target_angle_2, target_angle_3, 
                         target_angle_4, target_angle_5, -80);
            delay(800);
            
            grabState = STATE_LIFT;
            break;
            
        case STATE_LIFT:
            Serial.println("[ACTION] Lift");
            servo_control(target_angle_1, 20, 30, 0, 0, -80);
            delay(800);
            grabState = STATE_TRANSFER;
            break;
            
        case STATE_TRANSFER:
            Serial.println("[ACTION] Transfer");
            switch (colorType) {
                case 'B': transfer_blue(); break;
                case 'P': transfer_white(); break;
                case 'R': transfer_red(); break;
            }
            grabState = STATE_RESET;
            break;
            
        case STATE_RESET:
            Serial.println("[ACTION] Reset");
            servo_reset_begin();
            resetToIdle();
            break;
    }
}

// ==================== 分拣放置 ====================

void transfer_blue() {
    Serial.println("[TRANSFER] Blue -> Left");
    servo_control(-60, 30, 40, 0, 0, -80);
    delay(500);
    servo_control(-60, 45, 50, 0, 0, -80);
    delay(300);
    servo_control(-60, 45, 50, 0, 0, 20);
    delay(300);
}

void transfer_white() {
    Serial.println("[TRANSFER] White -> Left");
    servo_control(60, 30, 40, 0, 0, -80);
    delay(500);
    servo_control(60, 45, 50, 0, 0, -80);
    delay(300);
    servo_control(60, 45, 50, 0, 0, 20);
    delay(300);
}

void transfer_red() {
    Serial.println("[TRANSFER] Red -> Right");
    servo_control(60, 30, 40, 0, 0, -80);
    delay(500);
    servo_control(60, 45, 50, 0, 0, -80);
    delay(300);
    servo_control(60, 45, 50, 0, 0, 20);
    delay(300);
}