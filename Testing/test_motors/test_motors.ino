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

int pwm = 0;

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
  ledcWrite(channel2, (pwm-100));
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
  ledcWrite(channel, (pwm-100));
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

void setup() {
  // put your setup code here, to run once:
  motor_pin_config();
}

void loop() {
  pwm = 200;  
  slight_left(pwm);
  delay(2000);
  slight_right(pwm);
  delay(2000);
}
