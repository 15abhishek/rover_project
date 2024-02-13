
#include "AiEsp32RotaryEncoder.h"

#define FL_ROTARY_ENCODER_A_PIN 22
#define FL_ROTARY_ENCODER_B_PIN 34

#define FR_ROTARY_ENCODER_A_PIN 23
#define FR_ROTARY_ENCODER_B_PIN 35

#define RL_ROTARY_ENCODER_A_PIN 32
#define RL_ROTARY_ENCODER_B_PIN 33

#define RR_ROTARY_ENCODER_A_PIN 16
#define RR_ROTARY_ENCODER_B_PIN 17

#define ROTARY_ENCODER_BUTTON_PIN 23

#define ROTARY_ENCODER_STEPS 4 

uint16_t loop_delay = 10; // milliseconds
float delay_s;

int robot_width = 0.235;

int enc_FL = 0; // encoder tics
int enc_RL = 0; // encoder tics
int enc_FR = 0; // encoder tics
int enc_RR = 0; // encoder tics

float wheel_FL_ang_pos = 0; //radians
float wheel_FR_ang_pos = 0; //radians
float wheel_RL_ang_pos = 0; //radians
float wheel_RR_ang_pos = 0; //radians

int32_t enc_L = 0;         // encoder tics
float wheel_L_ang_pos = 0; // radians
float wheel_L_ang_vel = 0; // radians per second

int32_t enc_R = 0;         // encoder tics
float wheel_R_ang_pos = 0; // radians
float wheel_R_ang_vel = 0; // radians per second

float robot_angular_pos = 0; // radians
float robot_angular_vel = 0; // radians per second

float robot_x_pos = 0; // meters
float robot_y_pos = 0; // meters
float robot_x_vel = 0; // meters per second
float robot_y_vel = 0; // meters per second


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

void setup()
{
  Serial.begin(115200);
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

void loop()
{
  delay_s = (float)loop_delay / (float)1000;

  enc_FR = rotaryEncoderFR.readEncoder()
  enc_FL = rotaryEncoderFL.readEncoder()
  enc_RR = rotaryEncoderRR.readEncoder()
  enc_RL = rotaryEncoderRL.readEncoder()

  wheel_FL_ang_pos = 2 * 3.14 * enc_FL / enc_res;
  wheel_FR_ang_pos = 2 * 3.14 * enc_FR / enc_res;
  wheel_RL_ang_pos = 2 * 3.14 * enc_RL / enc_res;
  wheel_RR_ang_pos = 2 * 3.14 * enc_RR / enc_res;

  enc_L = (enc_FL + enc_RL) / 2 ;
  enc_R = (enc_FR + enc_RR) / 2 ;

  wheel_L_ang_pos = 2 * 3.14 * enc_L / enc_res;
  wheel_R_ang_pos = 2 * 3.14 * enc_R / enc_res; 

  wheel_L_ang_vel = ((2 * 3.14 * enc_L / enc_res) - wheel_L_ang_pos) / delay_s;
  wheel_R_ang_vel = ((2 * 3.14 * enc_R / enc_res) - wheel_R_ang_pos) / delay_s;

  robot_angular_vel = (((wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / (robot_width)) - robot_angular_pos) / delay_s;
  robot_angular_pos = (wheel_R_ang_pos - wheel_L_ang_pos) * wheel_radius / (robot_width);
  robot_x_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) * cos(robot_angular_pos);
  robot_y_vel = (wheel_L_ang_vel * wheel_radius + robot_angular_vel * robot_width / 2) * sin(robot_angular_pos);
  robot_x_pos = robot_x_pos + robot_x_vel * delay_s;
  robot_y_pos = robot_y_pos + robot_y_vel * delay_s;

  Serial.println(robot_x_pos);
  Serial.println(robot_y_pos);
  delay(loop_delay);
}