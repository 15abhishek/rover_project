#include "AiEsp32RotaryEncoder.h"

#define ROTARY_ENCODER_A_PIN 22
#define ROTARY_ENCODER_B_PIN 34
#define ROTARY_ENCODER_BUTTON_PIN 25

#define ROTARY_ENCODER_STEPS 4 

AiEsp32RotaryEncoder rotaryEncoder = AiEsp32RotaryEncoder(ROTARY_ENCODER_A_PIN, ROTARY_ENCODER_B_PIN, ROTARY_ENCODER_BUTTON_PIN, -1, ROTARY_ENCODER_STEPS);

void IRAM_ATTR readEncoderISR()
{
    rotaryEncoder.readEncoder_ISR();
}

void setup()
{
    Serial.begin(115200);
    rotaryEncoder.begin();
    rotaryEncoder.setup(readEncoderISR);
    rotaryEncoder.disableAcceleration();
//    rotaryEncoder.setBoundaries(0, 540, false); //minValue, maxValue, circleValues true|false (when max go to min and vice versa)
//    rotaryEncoder.setAcceleration(250);
}

void loop()
{
    if (rotaryEncoder.encoderChanged())
    {
        Serial.println(rotaryEncoder.readEncoder());
    }
    if (rotaryEncoder.isEncoderButtonClicked())
    {
        Serial.println("button pressed");
    }
}
