//Defining all motor pins
#define fl_motor1  26
#define fl_motor2  27
#define fl_pwm     25

#define fr_motor1  12
#define fr_motor2  13
#define fr_pwm     14

#define bl_motor1   2
#define bl_motor2   4
#define bl_pwm     15

#define br_motor1  18
#define br_motor2  19
#define br_pwm      5

//PWM configuration
const int freq = 900;
const int resolution = 8;
const int channel = 0;

int pwm = 0;

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

void forward(int pwm)
{
  
  ledcWrite(channel, pwm);
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
  digitalWrite(fl_motor1,LOW);
  digitalWrite(fl_motor2,HIGH);
  digitalWrite(fr_motor1,LOW);
  digitalWrite(fr_motor2,HIGH);
  digitalWrite(bl_motor1,LOW);
  digitalWrite(bl_motor2,HIGH);
  digitalWrite(br_motor1,LOW);
  digitalWrite(br_motor2,HIGH);
}

void setup() {
  // put your setup code here, to run once:
  motor_pin_config();
}

void loop() {
  pwm = 200;  
  forward(pwm);
  delay(1000);
  backward(pwm);
  delay(1000);
}
