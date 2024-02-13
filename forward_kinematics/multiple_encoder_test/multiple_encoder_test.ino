
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
//    if (rotaryEncoderFL.encoderChanged() or rotaryEncoderFR.encoderChanged() or rotaryEncoderRL.encoderChanged() or rotaryEncoderRR.encoderChanged())
//    {
        Serial.println(rotaryEncoderRR.readEncoder());
//    }
}
