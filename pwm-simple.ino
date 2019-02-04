#define serialSpeed 115200

#define bodyLeftMotorPWM 110
#define bodyRightMotorPWM 110

// motor one
#define enA 5
#define in1 4
#define in2 7
// motor two
#define enB 6
#define in3 8
#define in4 12

// LeftMotor
#define bodyLeftMotorEn enA
#define bodyLeftMotorForward in1
#define bodyLeftMotorBackward in2

// RightMotor
#define bodyRightMotorEn enB
#define bodyRightMotorForward in4
#define bodyRightMotorBackward in3

unsigned long loopTimeStart = 0;

void setup() {
  Serial.begin(serialSpeed); //Initialize the serial port

  pinMode(bodyLeftMotorEn, OUTPUT);
  pinMode(bodyLeftMotorForward, OUTPUT);
  pinMode(bodyLeftMotorBackward, OUTPUT);
  pinMode(bodyRightMotorEn, OUTPUT);
  pinMode(bodyRightMotorForward, OUTPUT);
  pinMode(bodyRightMotorBackward, OUTPUT);

  delay(3000);
  
  loopTimeStart = millis();
}

void loop() {
  unsigned long loopTime = 0;

  // set direction
  setBodyMotorLeftForward();
  setBodyMotorRightForward();
  
  // set speed
  analogWrite(bodyLeftMotorEn, bodyLeftMotorPWM);
  analogWrite(bodyRightMotorEn, bodyRightMotorPWM);
  // end loop
  loopTime = millis() - loopTimeStart;  
  loopTimeStart = millis();
}

void setBodyMotorLeftForward() {
  digitalWrite(bodyLeftMotorForward, HIGH);
  digitalWrite(bodyLeftMotorBackward, LOW);
}

void setBodyMotorRightForward() {
  digitalWrite(bodyRightMotorForward, HIGH);
  digitalWrite(bodyRightMotorBackward, LOW);
}
