#define FLencoderA 22
#define FLencoderB 34

#define FRencoderA  3
#define FRencoderB 35

#define RLencoderA 32
#define RLencoderB 33

#define RRencoderA 16
#define RRencoderB 17

volatile int FL_POS = 0;



void IRAM_ATTR ISR() {
  if(!digitalRead(FLencoderB)){
    FL_POS++;
  }
  else{
    FL_POS--;
  }
}

void setup() {
  pinMode(FLencoderA, INPUT_PULLUP);
  pinMode(FLencoderB, INPUT_PULLUP);
  pinMode(FRencoderA, INPUT_PULLUP);
  pinMode(FRencoderB, INPUT_PULLUP);
  pinMode(RLencoderA, INPUT_PULLUP);
  pinMode(RLencoderB, INPUT_PULLUP);
  pinMode(RRencoderA, INPUT_PULLUP);
  pinMode(RRencoderB, INPUT_PULLUP);
  
  attachInterrupt(FLencoderA, ISR, FALLING);
  
  Serial.begin(115200);
}

void loop() {
  Serial.println(FL_POS);
//  delay(100);
}
