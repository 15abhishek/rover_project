//Defining all motor pins
#define flMotor1  18
#define flMotor2  19
#define frMotor1  21
#define frMotor2  22
#define blMotor1  23
#define blMotor2  25
#define brMotor1  26
#define brMotor2  27

//PWM configuration
const int freq = 900;
const int resolution = 8;
const int channel = 0;

void motor_pin_config(void)
{
  pinMode(flMotor1,OUTPUT);
  pinMode(flMotor2,OUTPUT);
  pinMode(frMotor1,OUTPUT);
  pinMode(frMotor2,OUTPUT);
  pinMode(blMotor1,OUTPUT);
  pinMode(blMotor2,OUTPUT);
  pinMode(brMotor1,OUTPUT);
  pinMode(brMotor2,OUTPUT);
}

void forward()
{
  digitalWrite(flMotor1,HIGH);
  digitalWrite(flMotor2,LOW);
  digitalWrite(frMotor1,HIGH);
  digitalWrite(frMotor2,LOW);
  digitalWrite(blMotor1,HIGH);
  digitalWrite(blMotor2,LOW);
  digitalWrite(brMotor1,HIGH);
  digitalWrite(brMotor2,LOW);
}

void backward()
{
  digitalWrite(flMotor1,LOW);
  digitalWrite(flMotor2,HIGH);
  digitalWrite(frMotor1,LOW);
  digitalWrite(frMotor2,HIGH);
  digitalWrite(blMotor1,LOW);
  digitalWrite(blMotor2,HIGH);
  digitalWrite(brMotor1,LOW);
  digitalWrite(brMotor2,HIGH);
}

void setup() {
  // put your setup code here, to run once:
  motor_pin_config();
}

void loop() {
  // put your main code here, to run repeatedly:
  forward();
  delay(1000);
  backward();
  delay(1000);
}
