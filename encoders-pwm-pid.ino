#define serialSpeed 115200

#define bodyLeftMotorPwmRef 110
#define bodyRightMotorPwmRef 130
#define bodySteeringFactor 1

// encoder type
#define bodyEncoderSJ01 true
#define bodyEncoderHC020K false
#define bodyEncoderLM393 false

#if bodyEncoderSJ01
#define bodyEncoderSignal CHANGE
#define bodyEncoderInputSignal INPUT
#endif
#if bodyEncoderHC020K
#define bodyEncoderSignal FALLING
#define bodyEncoderInputSignal INPUT_PULLUP
#endif
#if bodyEncoderLM393
#define bodyEncoderSignal RISING
#define bodyEncoderInputSignal INPUT
#endif

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

// encoder LeftMotor
#define bodyEncoderLeftInt 0
#define bodyEncoderLeftFunction bodyEncoderLeftCounter
#define bodyEncoderLeftPin 2 // A pin the interrupt pin
#if bodyEncoderSJ01
#define bodyEncoderLeftPinB 9 // B pin: the digital pin
#endif
#define bodyEncoderLeftSignal bodyEncoderSignal
#define bodyEncoderLeftInputSignal bodyEncoderInputSignal

// encoder RightMotor
#define bodyEncoderRightInt 1
#define bodyEncoderRightFunction bodyEncoderRightCounter
#define bodyEncoderRightPin 3 // A pin: the interrupt pin
#if bodyEncoderSJ01
#define bodyEncoderRightPinB 10 // B pin: the digital pin
#endif
#define bodyEncoderRightSignal bodyEncoderSignal
#define bodyEncoderRightInputSignal bodyEncoderInputSignal

#include <PID_v1.h>
#define bodySteeringPidSetPointTarget 0
#define bodySteeringPidSampleTime 3
#define bodySteeringPidMinOutput -100
#define bodySteeringPidMaxOutput 100

#if bodyEncoderSJ01
byte bodyEncoderLeftPinLast;
boolean bodyEncoderLeftDirection; // the rotation direction
byte bodyEncoderRightPinLast;
boolean bodyEncoderRightDirection; //the rotation direction 
#endif

double bodySteeringPidInput;
double bodySteeringPidOutput; // Power supplied to the motor PWM value.
double bodySteeringPidSetPoint;
double bodySteeringPidKp=1, bbodySteeringPidKi=0.4, bodySteeringPidKd=0.001;

PID bodySteeringPid(&bodySteeringPidInput, &bodySteeringPidOutput, &bodySteeringPidSetPoint, bodySteeringPidKp, bbodySteeringPidKi, bodySteeringPidKd, REVERSE); 

volatile signed long bodyEncoderLeftTotalPulses = 0;
volatile signed long bodyEncoderRightTotalPulses = 0;

unsigned long loopTimeStart = 0;

void setup() {
  Serial.begin(serialSpeed); //Initialize the serial port

  pinMode(bodyLeftMotorEn, OUTPUT);
  pinMode(bodyLeftMotorForward, OUTPUT);
  pinMode(bodyLeftMotorBackward, OUTPUT);
  pinMode(bodyRightMotorEn, OUTPUT);
  pinMode(bodyRightMotorForward, OUTPUT);
  pinMode(bodyRightMotorBackward, OUTPUT);

  bodySteeringPidSetPoint = bodySteeringPidSetPointTarget; 
  bodySteeringPid.SetMode(AUTOMATIC); 
  bodySteeringPid.SetSampleTime(bodySteeringPidSampleTime); 
  bodySteeringPid.SetOutputLimits(bodySteeringPidMinOutput, bodySteeringPidMaxOutput);

#if bodyEncoderSJ01
  bodyEncoderLeftDirection = true; //default -> Forward  
  pinMode(bodyEncoderLeftPinB, bodyEncoderLeftInputSignal);  
#endif
  attachInterrupt(bodyEncoderLeftInt, bodyEncoderLeftFunction, bodyEncoderLeftSignal);

#if bodyEncoderSJ01
  bodyEncoderRightDirection = true; // default -> Forward  
  pinMode(bodyEncoderRightPinB, bodyEncoderRightInputSignal);  
#endif
  attachInterrupt(bodyEncoderRightInt, bodyEncoderRightFunction, bodyEncoderRightSignal);

  delay(3000);
  
  loopTimeStart = millis();

}

void loop() {
  unsigned long loopTime = 0;

  unsigned int bodyLeftMotorPwmOut = 0;
  unsigned int bodyRightMotorPwmOut = 0;
  signed long bodySteeringError = 0;
  boolean bodySteeringdPidResult = false;

  // set direction
  setBodyMotorLeftForward();
  setBodyMotorRightForward();

  // calculate pwm steering correction using pid
  bodySteeringError = abs(bodyEncoderLeftTotalPulses) - abs(bodyEncoderRightTotalPulses);
  bodySteeringPidInput = bodySteeringError;
  bodySteeringdPidResult = bodySteeringPid.Compute();

  bodyLeftMotorPwmOut = bodyLeftMotorPwmRef - (bodySteeringPidOutput * bodySteeringFactor);
  bodyRightMotorPwmOut = bodyRightMotorPwmRef + (bodySteeringPidOutput * bodySteeringFactor);
  
  // set speed
  analogWrite(bodyLeftMotorEn, bodyLeftMotorPwmOut);
  analogWrite(bodyRightMotorEn, bodyRightMotorPwmOut);

  if(bodySteeringdPidResult) {
    // display info
    //Serial.print(abs(bodyEncoderLeftTotalPulses)); Serial.print("\t");
    //Serial.print(abs(bodyEncoderRightTotalPulses)); Serial.print("\t");
    Serial.print((float)bodyLeftMotorPwmOut/10.0); Serial.print("\t"); // divided for scale display
    Serial.print((float)bodyRightMotorPwmOut/10.0); Serial.print("\t"); // divided for scale display
    Serial.print(bodySteeringError); Serial.print("\t");
    Serial.println("");
  }
  
  // end loop
  loopTime = millis() - loopTimeStart;  
  loopTimeStart = millis();
}
 
void bodyEncoderLeftCounter()
{
#if bodyEncoderSJ01
  int Lstate = digitalRead(bodyEncoderLeftPin);
  if((bodyEncoderLeftPinLast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(bodyEncoderLeftPinB);
    if(val == LOW && bodyEncoderLeftDirection)
    {
      bodyEncoderLeftDirection = false; //Reverse
    }
    else if(val == HIGH && !bodyEncoderLeftDirection)
    {
      bodyEncoderLeftDirection = true;  //Forward
    }
  }
  bodyEncoderLeftPinLast = Lstate;
 
  if(!bodyEncoderLeftDirection)
  {
    bodyEncoderLeftTotalPulses++;
  } else {
    bodyEncoderLeftTotalPulses--;
  }
#else
  bodyEncoderLeftTotalPulses ++;
#endif
}
 
void bodyEncoderRightCounter()
{
#if bodyEncoderSJ01
  int Lstate = digitalRead(bodyEncoderRightPin);
  if((bodyEncoderRightPinLast == LOW) && Lstate==HIGH)
  {
    int val = digitalRead(bodyEncoderRightPinB);
    if(val == LOW && bodyEncoderRightDirection)
    {
      bodyEncoderRightDirection = false; // Reverse
    }
    else if(val == HIGH && !bodyEncoderRightDirection)
    {
      bodyEncoderRightDirection = true;  // Forward
    }
  }
  bodyEncoderRightPinLast = Lstate;
 
  if(!bodyEncoderRightDirection)
  {
    bodyEncoderRightTotalPulses ++;  
  } else {
    bodyEncoderRightTotalPulses --;  
  }
#else
  bodyEncoderRightTotalPulses ++;  
#endif
}

void setBodyMotorLeftForward() {
  digitalWrite(bodyLeftMotorForward, HIGH);
  digitalWrite(bodyLeftMotorBackward, LOW);
}

void setBodyMotorRightForward() {
  digitalWrite(bodyRightMotorForward, HIGH);
  digitalWrite(bodyRightMotorBackward, LOW);
}
