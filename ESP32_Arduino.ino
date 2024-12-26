
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include <Adafruit_BNO08x.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>
#include <geometry_msgs/Quaternion.h>

#define LED_BUILTIN 2
#define USE_IMU 1
#define BNO08X_RESET -1


#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>


#define RXD2 16
#define TXD2 17

#define PUB_VEL 0   
#define USE_PID 1
#if USE_PID == 1
double trackAdjustValueL = 0.0;
double trackSetpointL = 0.0;
double trackErrorL = 0.0;
double trackAdjustValueR = 0.0;
double trackSetpointR = 0.0;
double trackErrorR = 0.0;

double Kp = 5.0;  
double Ki = 10.0; 
double Kd = 1.0;  

PID trackPIDLeft(&trackErrorL, &trackAdjustValueL, &trackSetpointL, Kp, Ki, Kd, DIRECT);
PID trackPIDRight(&trackErrorR, &trackAdjustValueR, &trackSetpointR, Kp, Ki, Kd, DIRECT);
#endif

ros::NodeHandle nh;
#if USE_IMU == 1
geometry_msgs::Quaternion quat_msg;
ros::Publisher quat_pub("quaternion", &quat_msg);
#endif

#define ENC_IN_LEFT_A 36
#define ENC_IN_RIGHT_A 34\

#define ENC_IN_LEFT_B 39
#define ENC_IN_RIGHT_B 35

#define enA 32
#define enB 33

#define ENA_CH 0
#define ENB_CH 1

#define ain1 26
#define ain2 25
#define bin1 27
#define bin2 14

boolean Direction_left = true;
boolean Direction_right = true;

#define encoder_minimum -2147483648
#define encoder_maximum 2147483647

#define MPU_INT 12

std_msgs::Int16 right_wheel_tick_count;
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

#if PUB_VEL == 1
std_msgs::Float32 left_vel_pub;
ros::Publisher leftvelPub("velLeft", &left_vel_pub);
std_msgs::Float32 right_vel_pub;
ros::Publisher rightvelPub("velRight", &right_vel_pub);
#endif

const int INTERVAL = 30;
long previousMillis = 0;
long currentMillis = 0;

#define WHEEL_RADIUS (0.033)
#define WHEEL_DIAMETER (WHEEL_RADIUS * 2)

#define TICKS_PER_REVOLUTION (1860.0)
#define TICKS_PER_METER (TICKS_PER_REVOLUTION / (2.0 * 3.141592 * WHEEL_RADIUS))
#define WHEEL_BASE (0.160)


#define K_P 1125.0
#define K_b 3.5
#define PWM_MIN 30.0   
#define PWM_MAX 240.0  
#define K_bias 0.0    

#define PWM_TURN (PWM_MIN)
#define PWM_INCREMENT 1

float velLeftWheel = 0.0;
float velRightWheel = 0.0;
float vLeft = 0.0;
float vRight = 0.0;

int pwmLeftReq = 0;
int pwmRightReq = 0;

float lastCmdVelReceived = 0.0;

#if USE_IMU == 1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#endif

void IRAM_ATTR left_wheel_tick() {
  int val = digitalRead(ENC_IN_LEFT_B);

  if (val == LOW) {
    Direction_left = true;  
  } else {
    Direction_left = false;  
  }

  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    } else {
      left_wheel_tick_count.data++;
    }
  } else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    } else {
      left_wheel_tick_count.data--;
    }
  }
}

void IRAM_ATTR right_wheel_tick() {
  int val = digitalRead(ENC_IN_RIGHT_B);

  if (val == LOW) {
    Direction_right = false;  // Reverse
  } else {
    Direction_right = true;  // Forward
  }

  if (Direction_right) {

    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    } else {
      right_wheel_tick_count.data++;
    }
  } else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    } else {
      right_wheel_tick_count.data--;
    }
  }
}

void calc_vel_left_wheel() {

  static float prevTime = 0.0;

  static int prevLeftCount = 0;

  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;

  if (numOfTicks > 30000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  velLeftWheel = float(numOfTicks) / TICKS_PER_METER / ((millis() / 1000.0) - prevTime);
#if PUB_VEL == 1
  left_vel_pub.data = velLeftWheel;
#endif
  prevLeftCount = left_wheel_tick_count.data;

  prevTime = (millis() / 1000.0);
}

void calc_vel_right_wheel() {

  static float prevTime = 0.0;

  static int prevRightCount = 0;

  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;

  if (numOfTicks > 30000) {
    numOfTicks = 0 - (65535 - numOfTicks);
  }

  velRightWheel = float(numOfTicks) / TICKS_PER_METER / ((millis() / 1000.0) - prevTime);
#if PUB_VEL == 1
  right_vel_pub.data = velRightWheel;
#endif
  prevRightCount = right_wheel_tick_count.data;

  // Update the timestamp
  prevTime = (millis() / 1000.0);
}

void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
  float vLeft, vRight;

  lastCmdVelReceived = (millis() / 1000.0);

  vLeft = cmdVel.linear.x - cmdVel.angular.z * WHEEL_BASE / 2.0;
  vRight = cmdVel.linear.x + cmdVel.angular.z * WHEEL_BASE / 2.0;

  if (vLeft >= 0.0) {
    pwmLeftReq = int(K_P * vLeft + K_b + K_bias);
  } else {
    pwmLeftReq = int(K_P * vLeft - K_b - K_bias);
  }
  if (vRight >= 0.0) {
    pwmRightReq = K_P * vRight + K_b;
  } else {
    pwmRightReq = K_P * vRight - K_b;
  }

  if (abs(pwmLeftReq) < PWM_MIN) {
    pwmLeftReq = 0;
  }
  if (abs(pwmRightReq) < PWM_MIN) {
    pwmRightReq = 0;
  }

#if USE_PID == 1
  trackPIDLeft.SetMode(MANUAL);
  trackAdjustValueL = 0.0;
  trackErrorL = 0.0;
  trackPIDLeft.SetMode(AUTOMATIC);

  trackPIDRight.SetMode(MANUAL);
  trackAdjustValueR = 0.0;
  trackErrorR = 0.0;
  trackPIDRight.SetMode(AUTOMATIC);
#endif  //USE_PID
}

void set_pwm_values() {

  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;

  static bool stopped = false;
  if (((pwmLeftReq * velLeftWheel < 0) && (pwmLeftOut != 0)) || ((pwmRightReq * velRightWheel < 0) && (pwmRightOut != 0))) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  if (pwmLeftReq > 0) {  
    digitalWrite(ain1, HIGH);
    digitalWrite(ain2, LOW);
  } else if (pwmLeftReq < 0) { 
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, HIGH);
  } else if (pwmLeftReq == 0 && pwmLeftOut == 0) {  
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, LOW);
  } else {  
    digitalWrite(ain1, LOW);
    digitalWrite(ain2, LOW);
  }

  if (pwmRightReq > 0) {  
    digitalWrite(bin1, HIGH);
    digitalWrite(bin2, LOW);
  } else if (pwmRightReq < 0) {  
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, HIGH);
  } else if (pwmRightReq == 0 && pwmRightOut == 0) {  
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, LOW);
  } else {  
    digitalWrite(bin1, LOW);
    digitalWrite(bin2, LOW);
  }

  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  } else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;

  } else {
#if USE_PID == 1
    if (pwmLeftReq != 0) {
      trackErrorL = (velLeftWheel - vLeft) * 100.0;
      if (trackPIDLeft.Compute())  //true if PID has triggered
        pwmLeftOut += trackAdjustValueL;
    }
#endif
  }

  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  } else if (abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  } else {
#if USE_PID == 1
    if (pwmRightReq != 0) {
      trackErrorR = (velRightWheel - vRight) * 100.0;
      if (trackPIDRight.Compute())  //true if PID has triggered
        pwmRightOut += trackAdjustValueR;
    }
#endif
  }

  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;

  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;

  if (pwmLeftOut > 0) {
    ledcWrite(ENA_CH, pwmLeftOut);
  } else {
    ledcWrite(ENA_CH, -pwmLeftOut);
  }
  if (pwmRightOut > 0) {
    ledcWrite(ENB_CH, pwmRightOut);
  } else {
    ledcWrite(ENB_CH, -pwmRightOut);
  }
}



ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values);

void setup() {
#if (DEBUG == 1)
  Serial2.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
#endif

  pinMode(ENC_IN_LEFT_A, INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B, INPUT);
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);

  pinMode(ain1, OUTPUT);
  pinMode(ain2, OUTPUT);
  pinMode(bin1, OUTPUT);
  pinMode(bin2, OUTPUT);

  digitalWrite(ain1, LOW);
  digitalWrite(ain2, LOW);
  digitalWrite(bin1, LOW);
  digitalWrite(bin2, LOW);

  ledcSetup(ENA_CH, 5000, 8);  
  ledcSetup(ENB_CH, 5000, 8);  

  ledcAttachPin(enA, ENA_CH);
  ledcAttachPin(enB, ENB_CH);

  // Set the motor speed
  ledcWrite(ENA_CH, 0);
  ledcWrite(ENB_CH, 0);

  DEBUG_PRINTLN("SYSTEM Started");


#if USE_IMU == 1
  mpu_setup();
#endif


  // ROS Setup
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);

#if USE_IMU == 1
  nh.advertise(quat_pub);
#endif

#if PUB_VEL == 1
  nh.advertise(rightvelPub);
  nh.advertise(leftvelPub);
#endif

  while (!nh.connected()) {
    nh.spinOnce();
  }

  DEBUG_PRINTLN("\nROS connected");
#if USE_PID == 1
  trackPIDLeft.SetMode(AUTOMATIC);
  trackPIDLeft.SetSampleTime(200);
  trackPIDLeft.SetOutputLimits(-20, 20);
  trackPIDRight.SetMode(AUTOMATIC);
  trackPIDRight.SetSampleTime(200);
  trackPIDRight.SetOutputLimits(-20, 20);
#endif
}

void loop() {
  nh.spinOnce();

  currentMillis = millis();

  if (currentMillis - previousMillis > INTERVAL) {
    previousMillis = currentMillis;

    DEBUG_PRINT("LT:");
    DEBUG_PRINT(int(left_wheel_tick_count.data));
    DEBUG_PRINT(",RT:");
    DEBUG_PRINTLN(int(right_wheel_tick_count.data));

    // Publish tick counts to topics
    leftPub.publish(&left_wheel_tick_count);
    rightPub.publish(&right_wheel_tick_count);
#if PUB_VEL == 1
    leftvelPub.publish(&left_vel_pub);
    rightvelPub.publish(&right_vel_pub);
#endif
    calc_vel_right_wheel();
    calc_vel_left_wheel();

    mpu_loop();
  }

  if ((millis() / 1000) - lastCmdVelReceived > 1) {
    pwmLeftReq = 0;
    pwmRightReq = 0;
  }

  set_pwm_values();
}

void setReports(void) {
  if (! bno08x.enableReport(SH2_GAME_ROTATION_VECTOR)) {
  }
}

void mpu_setup() {
 if (!bno08x.begin_I2C()) {  
    while (1) { delay(10); }
 }
  setReports();
  delay(100);
  
}

void mpu_loop() {
  delay(10);
  if (bno08x.wasReset()) {
    setReports();
  }
  if (! bno08x.getSensorEvent(&sensorValue)) {
    return;
  }
  switch (sensorValue.sensorId) { 
    case SH2_GAME_ROTATION_VECTOR:
      quat_msg.w = sensorValue.un.gameRotationVector.real;
      quat_msg.x = sensorValue.un.gameRotationVector.i;
      quat_msg.y = sensorValue.un.gameRotationVector.j;
      quat_msg.z = sensorValue.un.gameRotationVector.k;
      quat_pub.publish(&quat_msg);
      break;
  }
}