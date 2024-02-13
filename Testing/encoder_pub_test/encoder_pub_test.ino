#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "AiEsp32RotaryEncoder.h"

#include <geometry_msgs/msg/twist.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only avaible for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

rcl_publisher_t publisher;
geometry_msgs__msg__Twist msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 13

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

AiEsp32RotaryEncoder rotaryEncoderFL = AiEsp32RotaryEncoder(FL_ROTARY_ENCODER_A_PIN, FL_ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoder rotaryEncoderFR = AiEsp32RotaryEncoder(FR_ROTARY_ENCODER_A_PIN, FR_ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoder rotaryEncoderRL = AiEsp32RotaryEncoder(RL_ROTARY_ENCODER_A_PIN, RL_ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoder rotaryEncoderRR = AiEsp32RotaryEncoder(RR_ROTARY_ENCODER_A_PIN, RR_ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);

volatile int encFL = 0;
volatile int encFR = 0;
volatile int encRL = 0;
volatile int encRR = 0;

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
}

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.linear.x = 0.0;
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
  
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
  }
}

void setup() {

  encoder_config();
  
  set_microros_wifi_transports("WIFI", "a9111707000", "192.168.43.225", 8888);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "test_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "encoders"));  
  
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;

  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;  
}

void loop() {
  read_encoder_values();
  
  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.linear.x = encFL;
    msg.linear.y = encFR;
    msg.linear.z = encRL;
  
    msg.angular.x = encRR;
    msg.angular.y = 0.0;
    msg.angular.z = 0.0;
}
