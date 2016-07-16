// Adapted from sketch By JohnChi
#include <Wire.h>
int mpu = 0x68; // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int count, countMean;
int curMaxVal,curMinVal,MaxVal,MinVal;
int amplitude,amplitudeMean,threshold;
volatile int calibrateSignal,resetSignal;
long prevX, prevY, prevZ;
long curX, curY, curZ;
long changeX, changeY, changeZ;
int calibratePin = 2;
int resetPin = 3;
int redLED = 4;
int blueLED = 5;
int greenLED = 6;
int buzzerPin = 8;
int fallNotice;
boolean childDeath;
void setup(){
  Wire.begin();
  Wire.beginTransmission(mpu);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up the MPU-6050
  Wire.endTransmission(true);
  Serial.begin(9600);
  count =1;
  countMean = 5;
  curMaxVal = AcY;
  MaxVal = AcY;
  curMinVal = AcY;
  MaxVal = AcY;
  amplitude = 0;
  amplitudeMean = 0;
  threshold = 0;
  childDeath = false;
  fallNotice = 300;
  attachInterrupt(digitalPinToInterrupt(calibratePin),calibrateHandler,CHANGE);
  attachInterrupt(digitalPinToInterrupt(resetPin),resetHandler,CHANGE);
  calibrateSignal = LOW;
  pinMode(buzzerPin,OUTPUT);
  pinMode(redLED,OUTPUT);
  pinMode(blueLED,OUTPUT);
  pinMode(greenLED,OUTPUT);
  prevX = Wire.read() << 8|Wire.read();
  prevY = Wire.read() << 8|Wire.read();
  prevZ = Wire.read() << 8|Wire.read();
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();
}
void loop(){
  Wire.beginTransmission(mpu);
  Wire.write(0x3B);                              // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(mpu,14,true);                // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();              // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();              // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();              // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();              // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();              // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();              // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();              // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  delay(2);

  //if(calibrateSignal == HIGH){
  if(countMean < 5){
      if(count < 1000){
        curMaxVal = AcY;
        curMinVal = AcY;
        if(curMaxVal > MaxVal){
          MaxVal = curMaxVal;
        }
        if(curMinVal < MinVal){
          MinVal = curMinVal;
        }
        count++;
      }
      else if(count == 1000){
        if(countMean!=0){
          amplitude = MaxVal - MinVal;
          amplitudeMean = amplitude + amplitudeMean;
        }
        Serial.print("mean: ");
        Serial.println(amplitudeMean);
        Serial.print("Max: ");
        Serial.println(MaxVal);
        Serial.print("Min: ");
        Serial.println(MinVal);
        countMean++;
        count = 0;
        curMaxVal = AcY;
        MaxVal = AcY;
        curMinVal = AcY;
        MinVal = AcY;
      }
    noTone(buzzerPin);
    digitalWrite(blueLED,HIGH);
    digitalWrite(redLED,LOW);
    digitalWrite(greenLED,LOW);
  }
  else{
    threshold = amplitudeMean/(countMean-1);
    if(count<1000){
      curMaxVal =AcY;
      curMinVal = AcY;
      if(curMaxVal > MaxVal){
        MaxVal = curMaxVal;
      }
      if(curMinVal < MinVal){
        MinVal = curMinVal;
      }
      if(childDeath == true){
        tone(buzzerPin,1000);
        digitalWrite(redLED,HIGH);
        digitalWrite(blueLED,LOW);
        digitalWrite(greenLED,LOW);
      }
      if(childDeath == false){
        noTone(8);
        digitalWrite(greenLED,HIGH);
        digitalWrite(redLED,LOW);
        digitalWrite(blueLED,LOW);
      }
      count++;
    }
    else if(count==1000){
      amplitude = MaxVal - MinVal;
      Serial.print("threshold: ");
      Serial.println(threshold);
      Serial.print("amplitude:");
      Serial.println(amplitude);
      Serial.print("Max: ");
      Serial.println(MaxVal);
      Serial.print("Min: ");
      Serial.println(MinVal);
      if(threshold - amplitude > (threshold/2)){
        if(threshold!= 0){
          Serial.println("YOUR CHILD IS DYING!");
          tone(buzzerPin,1000);
          digitalWrite(redLED,HIGH);
          digitalWrite(blueLED,LOW);
          digitalWrite(greenLED,LOW);
          childDeath =true;
        }  
      }
      else{
        noTone(8);
        digitalWrite(greenLED,HIGH);
        digitalWrite(redLED,LOW);
        digitalWrite(blueLED,LOW);
        childDeath = false;
      }
      count = 0;
      curMaxVal = AcY;
      MaxVal = AcY;
      curMinVal = AcY;
      MinVal = AcY;
    }
    
  }

  curX = AcX;
  curY = AcY;
  curZ = AcZ;

  changeX = abs(curX - prevX);
  changeY = abs(curY - prevY);
  changeZ = abs(curZ - prevZ);
 /* if(changeX > 15000){
    Serial.println("DEAD");
    tone(buzzerPin,2000,8);
  }

  if(changeY > 15000){
    Serial.println("DEAD");
    tone(buzzerPin,2000,8);
  }

  if(changeZ > 15000){
    Serial.println("DEAD");
    tone(buzzerPin,2000,8);
  }
*/
  if(changeX > 15000){
   // if(changeY > 15000){
      //if(changeZ > 15000)
        fallNotice = 0;
     // }
   // }
  }

  if(changeY > 15000){
    fallNotice = 0;
  }

  if(changeZ > 15000){
    fallNotice = 0;
  }
  if(fallNotice < 300){
    tone(buzzerPin,700);
      //Serial.println("OH NO BABY JUST FELL");
    fallNotice ++;
  }
  else{
    noTone(buzzerPin);
  }
  prevX = AcX;
  prevY = AcY;
  prevZ = AcZ;

  
}

void calibrateHandler(){
  //calibrateSignal = digitalRead(calibratePin);
  countMean = 0;
}

void resetHandler(){
  count =1;
  countMean = 5;
  curMaxVal = AcY;
  MaxVal = AcY;
  curMinVal = AcY;
  MaxVal = AcY;
  amplitude = 0;
  amplitudeMean = 0;
  threshold = 0;
  childDeath = false;
}

