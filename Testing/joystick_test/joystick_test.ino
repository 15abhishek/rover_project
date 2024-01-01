#define xAxis A0
#define yAxis A1
#define sw    A7

int x_val = 0;
int y_val = 0;
int sw_val = 0;

void setup() {
  pinMode(xAxis, INPUT);
  pinMode(yAxis, INPUT);
  pinMode(sw, INPUT);

  Serial.begin(115200);
}

void loop() {
  x_val = map(constrain(analogRead(xAxis),0,858),0,858,-255,255);
  y_val = map(constrain(analogRead(yAxis),0,895),0,895,-255,255);
  if(analogRead(sw)== 0){
    sw_val = 1;
  }
  else{
    sw_val = 0;
  }
    
//  Serial.print("x-Axis: ");
//  Serial.print(x_val);
//  Serial.print(" , y-Axis: ");
//  Serial.print(y_val);
  Serial.print(" , Switch: "); 
  Serial.print(sw_val);
  Serial.println();
//  delay(500);
}
