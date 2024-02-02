#include "AiEsp32RotaryEncoder.h"

#define FL_ROTARY_ENCODER_A_PIN 36
#define FL_ROTARY_ENCODER_B_PIN 39

#define FR_ROTARY_ENCODER_A_PIN 34
#define FR_ROTARY_ENCODER_B_PIN 35

#define RL_ROTARY_ENCODER_A_PIN 32
#define RL_ROTARY_ENCODER_B_PIN 33

#define RR_ROTARY_ENCODER_A_PIN 16
#define RR_ROTARY_ENCODER_B_PIN 17

#define ROTARY_ENCODER_STEPS 4 

AiEsp32RotaryEncoder rotaryEncoderFL = AiEsp32RotaryEncoder(FL_ROTARY_ENCODER_A_PIN, FL_ROTARY_ENCODER_B_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoder rotaryEncoderFR = AiEsp32RotaryEncoder(FR_ROTARY_ENCODER_A_PIN, FR_ROTARY_ENCODER_B_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoder rotaryEncoderRL = AiEsp32RotaryEncoder(RL_ROTARY_ENCODER_A_PIN, RL_ROTARY_ENCODER_B_PIN, -1, ROTARY_ENCODER_STEPS);
AiEsp32RotaryEncoder rotaryEncoderRR = AiEsp32RotaryEncoder(RR_ROTARY_ENCODER_A_PIN, RR_ROTARY_ENCODER_B_PIN, -1, ROTARY_ENCODER_STEPS);


void IRAM_ATTR readEncoderISR()
{
    rotaryEncoderFL.readEncoder_ISR();
    rotaryEncoderFR.readEncoder_ISR();
    rotaryEncoderRL.readEncoder_ISR();
    rotaryEncoderRR.readEncoder_ISR();
}

void setup()
{
    Serial.begin(115200);
    rotaryEncoderFL.begin();
    rotaryEncoderFL.setup(readEncoderISR);
    rotaryEncoderFL.disableAcceleration();
    
    rotaryEncoderFR.begin();
    rotaryEncoderFR.setup(readEncoderISR);
    rotaryEncoderFR.disableAcceleration();
    
    rotaryEncoderRL.begin();
    rotaryEncoderRL.setup(readEncoderISR);
    rotaryEncoderRL.disableAcceleration();
    
    rotaryEncoderRR.begin();
    rotaryEncoderRR.setup(readEncoderISR);
    rotaryEncoderRR.disableAcceleration();
}

void loop()
{
    if (rotaryEncoderFL.encoderChanged() or rotaryEncoderFR.encoderChanged() or rotaryEncoderRL.encoderChanged() or rotaryEncoderRR.encoderChanged())
    {
        Serial.println(rotaryEncoderFL.readEncoder());
    }
}
