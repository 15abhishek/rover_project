#include <AiEsp32RotaryEncoderNumberSelector.h>
#include <AiEsp32RotaryEncoder.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/int32_multi_array.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

//Defining all motor pins
#define fl_motor1   2
#define fl_motor2   4
#define fl_pwm     15

#define fr_motor1  14
#define fr_motor2  13
#define fr_pwm     12

#define bl_motor1  27
#define bl_motor2  26
#define bl_pwm     25

#define br_motor1   5
#define br_motor2  18
#define br_pwm     19

#define FL_ROTARY_ENCODER_A_PIN 22
#define FL_ROTARY_ENCODER_B_PIN 34

#define FR_ROTARY_ENCODER_A_PIN 23
#define FR_ROTARY_ENCODER_B_PIN 35

#define RL_ROTARY_ENCODER_A_PIN 16
#define RL_ROTARY_ENCODER_B_PIN 17

#define RR_ROTARY_ENCODER_A_PIN 32
#define RR_ROTARY_ENCODER_B_PIN 33

#define ROTARY_ENCODER_BUTTON_PIN 23

#define ROTARY_ENCODER_STEPS 4 

#define LED_PIN 13

//PWM configuration
const int freq = 900;
const int resolution = 8;
const int channel = 0;
const int channel2 = 0;

volatile int encFL = 0;
volatile int encFR = 0;
volatile int encRL = 0;
volatile int encRR = 0;

int enc[4] = {0, 0, 0, 0};

// Motor Functions
void motor_pin_config(void)
{
  pinMode(fl_pwm,OUTPUT);
  pinMode(fr_pwm,OUTPUT);
  pinMode(bl_pwm,OUTPUT);
  pinMode(br_pwm,OUTPUT);

  ledcSetup(channel, freq, resolution);
  ledcAttachPin(fl_pwm, channel);
  ledcAttachPin(fr_pwm, channel);
  ledcAttachPin(bl_pwm, channel);
  ledcAttachPin(br_pwm, channel);
  
  pinMode(fl_motor1,OUTPUT);
  pinMode(fl_motor2,OUTPUT);
  pinMode(fr_motor1,OUTPUT);
  pinMode(fr_motor2,OUTPUT);
  pinMode(bl_motor1,OUTPUT);
  pinMode(bl_motor2,OUTPUT);
  pinMode(br_motor1,OUTPUT);
  pinMode(br_motor2,OUTPUT);
}

void move_bot(int pwm, int dir)
{
  if(pwm > 30){
    if(dir > -30 and dir < 30){
    forward(pwm);
    }
//    else if(dir > 50){
//      slight_right(dir);
//    }
//    else if(dir < -50){
//      slight_left(-dir);
//    }
  }
  else if(pwm < -30 and dir > -50 and dir < 30){
    backward(-pwm);
  }
  else if(pwm > -30 and pwm < 30 and dir < -30){
    left(-dir);
  }
  else if(pwm > -30 and pwm < 30 and dir > 30){
    right(dir);
  }
  else{
    stop_bot();
  }
}

void forward(int pwm)
{
  
  ledcWrite(channel, pwm);
  ledcWrite(channel2, pwm);
  digitalWrite(fl_motor1,HIGH);
  digitalWrite(fl_motor2,LOW);
  digitalWrite(fr_motor1,HIGH);
  digitalWrite(fr_motor2,LOW);
  digitalWrite(bl_motor1,HIGH);
  digitalWrite(bl_motor2,LOW);
  digitalWrite(br_motor1,HIGH);
  digitalWrite(br_motor2,LOW);
}


void backward(int pwm)
{
  ledcWrite(channel, pwm);
  ledcWrite(channel2, pwm);
  digitalWrite(fl_motor1,LOW);
  digitalWrite(fl_motor2,HIGH);
  digitalWrite(fr_motor1,LOW);
  digitalWrite(fr_motor2,HIGH);
  digitalWrite(bl_motor1,LOW);
  digitalWrite(bl_motor2,HIGH);
  digitalWrite(br_motor1,LOW);
  digitalWrite(br_motor2,HIGH);
}

void right(int pwm)
{
  ledcWrite(channel, pwm);
  ledcWrite(channel2, pwm);
  digitalWrite(fl_motor1,HIGH);
  digitalWrite(fl_motor2,LOW);
  digitalWrite(fr_motor1,LOW);
  digitalWrite(fr_motor2,HIGH);
  digitalWrite(bl_motor1,HIGH);
  digitalWrite(bl_motor2,LOW);
  digitalWrite(br_motor1,LOW);
  digitalWrite(br_motor2,HIGH);
}

void left(int pwm)
{
  ledcWrite(channel, pwm);
  ledcWrite(channel2, pwm);
  digitalWrite(fl_motor1,LOW);
  digitalWrite(fl_motor2,HIGH);
  digitalWrite(fr_motor1,HIGH);
  digitalWrite(fr_motor2,LOW);
  digitalWrite(bl_motor1,LOW);
  digitalWrite(bl_motor2,HIGH);
  digitalWrite(br_motor1,HIGH);
  digitalWrite(br_motor2,LOW);
}

void slight_right(int pwm)
{
  ledcWrite(channel, pwm);
  ledcWrite(channel2, (0));
  digitalWrite(fl_motor1,HIGH);
  digitalWrite(fl_motor2,LOW);
  digitalWrite(fr_motor1,HIGH);
  digitalWrite(fr_motor2,LOW);
  digitalWrite(bl_motor1,HIGH);
  digitalWrite(bl_motor2,LOW);
  digitalWrite(br_motor1,HIGH);
  digitalWrite(br_motor2,LOW);
}

void slight_left(int pwm)
{
  ledcWrite(channel, (0));
  ledcWrite(channel2, pwm);
  digitalWrite(fl_motor1,HIGH);
  digitalWrite(fl_motor2,LOW);
  digitalWrite(fr_motor1,HIGH);
  digitalWrite(fr_motor2,LOW);
  digitalWrite(bl_motor1,HIGH);
  digitalWrite(bl_motor2,LOW);
  digitalWrite(br_motor1,HIGH);
  digitalWrite(br_motor2,LOW);
}

void stop_bot()
{
  ledcWrite(channel, 0);
  ledcWrite(channel2, 0);
  digitalWrite(fl_motor1,HIGH);
  digitalWrite(fl_motor2,HIGH);
  digitalWrite(fr_motor1,HIGH);
  digitalWrite(fr_motor2,HIGH);
  digitalWrite(bl_motor1,HIGH);
  digitalWrite(bl_motor2,HIGH);
  digitalWrite(br_motor1,HIGH);
  digitalWrite(br_motor2,HIGH);
}

// Encoder Functions

AiEsp32RotaryEncoder rotaryEncoderFL = AiEsp32RotaryEncoder(FL_ROTARY_ENCODER_A_PIN, FL_ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoder rotaryEncoderFR = AiEsp32RotaryEncoder(FR_ROTARY_ENCODER_A_PIN, FR_ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoder rotaryEncoderRL = AiEsp32RotaryEncoder(RL_ROTARY_ENCODER_A_PIN, RL_ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoder rotaryEncoderRR = AiEsp32RotaryEncoder(RR_ROTARY_ENCODER_A_PIN, RR_ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);

void IRAM_ATTR FLreadEncoderISR()
{
    rotaryEncoderFL.readEncoder_ISR();
}

void IRAM_ATTR FRreadEncoderISR()
{
    rotaryEncoderFR.readEncoder_ISR();
}

void IRAM_ATTR RLreadEncoderISR()
{
    rotaryEncoderRL.readEncoder_ISR();
}

void IRAM_ATTR RRreadEncoderISR()
{
    rotaryEncoderRR.readEncoder_ISR();
}

void encoder_config()
{
    rotaryEncoderFL.begin();
    rotaryEncoderFL.setup(FLreadEncoderISR);
    rotaryEncoderFL.disableAcceleration();
    
    rotaryEncoderFR.begin();
    rotaryEncoderFR.setup(FRreadEncoderISR);
    rotaryEncoderFR.disableAcceleration();
    
    rotaryEncoderRL.begin();
    rotaryEncoderRL.setup(RLreadEncoderISR);
    rotaryEncoderRL.disableAcceleration();
    
    rotaryEncoderRR.begin();
    rotaryEncoderRR.setup(RRreadEncoderISR);
    rotaryEncoderRR.disableAcceleration();
}

void read_encoder_values()
{
  encFL = rotaryEncoderFL.readEncoder();
  encFR = rotaryEncoderFR.readEncoder();
  encRL = rotaryEncoderRL.readEncoder();
  encRR = rotaryEncoderRR.readEncoder();

  // Update the global enc array
  enc[0] = encFL;
  enc[1] = encFR;
  enc[2] = encRL;
  enc[3] = encRR;
}


rcl_publisher_t publisher;
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
std_msgs__msg__Int32MultiArray arr;
geometry_msgs__msg__PoseWithCovariance pose;
geometry_msgs__msg__TwistWithCovariance twist;
nav_msgs__msg__Odometry odom;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

void error_loop(){
//  while(1){
//    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
//    Serial.println("Error!!");
//    delay(100);
//  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    for (size_t i = 0; i < 4; ++i)
    {
        arr.data.data[i] = enc[i];
    }
  }
} 

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  Serial.println("msg received");
}

void setup() {
  Serial.begin(115200);
  
  motor_pin_config();

  encoder_config();

  set_microros_wifi_transports("WIFI", "a9111707000", "192.168.0.105", 8888);

//  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);  
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "bot_node", "", &support));

  // create publisher
    RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "enc_values"));

    arr.data.data = (int32_t *)malloc(sizeof(int32_t) * 4);
    arr.data.size = 4;

    // Populate the message data with encoder values
    for (size_t i = 0; i < 4; ++i)
    {
        arr.data.data[i] = enc[i];
    }
    
  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

void loop() {
  
  read_encoder_values();
  
//  Serial.println(enc[);
    
    for (size_t i = 0; i < 4; ++i)
    {
        arr.data.data[i] = enc[i];
        Serial.println(enc[i]);
    }
    
  RCSOFTCHECK(rcl_publish(&publisher, &arr, NULL));
  
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  int pwm = int(msg.linear.x);
  int dir = int(msg.angular.z);

  move_bot(pwm, dir);
}
