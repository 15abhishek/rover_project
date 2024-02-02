#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h>

rcl_subscription_t subscriber;
rcl_subscription_t best_effort_subscriber;
geometry_msgs__msg__Twist msg;
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;


#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}

/////////////////////////////////////////////////////////////////////////////////////////////
//Defining all motor pins
#define fl_motor1   2
#define fl_motor2   4
#define fl_pwm     15

#define fr_motor1  12
#define fr_motor2  13
#define fr_pwm     14

#define bl_motor1  26
#define bl_motor2  27
#define bl_pwm     25

#define br_motor1  18
#define br_motor2  19
#define br_pwm      5

//PWM configuration
const int freq = 900;
const int resolution = 8;
const int channel = 0;
const int channel2 = 0;

/////////////////////////////////////////////////////////////////////////////////////////////

void motor_pin_config(void)
{
  pinMode(fl_pwm,OUTPUT);
  pinMode(fr_pwm,OUTPUT);
  pinMode(bl_pwm,OUTPUT);
  pinMode(br_pwm,OUTPUT);

  ledcSetup(channel, freq, resolution);
  ledcAttachPin(fl_pwm, channel);
  ledcAttachPin(fr_pwm, channel2);
  ledcAttachPin(bl_pwm, channel);
  ledcAttachPin(br_pwm, channel2);
  
  pinMode(fl_motor1,OUTPUT);
  pinMode(fl_motor2,OUTPUT);
  pinMode(fr_motor1,OUTPUT);
  pinMode(fr_motor2,OUTPUT);
  pinMode(bl_motor1,OUTPUT);
  pinMode(bl_motor2,OUTPUT);
  pinMode(br_motor1,OUTPUT);
  pinMode(br_motor2,OUTPUT);
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


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    Serial.println("Error!!");
    delay(100);
  }
}

//twist message cb
void subscription_callback(const void *msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  Serial.println("msg received");
}

void setup() {
//  Serial.begin(115200);
  motor_pin_config();
  
  set_microros_wifi_transports("WIFI", "a9111707000", "192.168.43.225", 8888);

//  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);  
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

   //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "bot_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_best_effort(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

}

void loop() {
//  delay(100);
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
  int pwm = int(msg.linear.x);
  int dir = int(msg.angular.z);

  if(pwm > 50){
    if(dir > -50 and dir < 50){
    forward(pwm);
    }
    else if(dir > 50){
      slight_right(dir);
    }
    else if(dir < -50){
      slight_left(-dir);
    }
  }
  else if(pwm < -50 and dir > -50 and dir < 50){
    backward(-pwm);
  }
  else if(pwm > -50 and pwm < 50 and dir < -50){
    left(-dir);
  }
  else if(pwm > -50 and pwm < 50 and dir > 50){
    right(dir);
  }
  else{
    stop_bot();
  }
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
