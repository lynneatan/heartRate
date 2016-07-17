// Adapted from BME261
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

//Heart Rate Varibles 
volatile int samplesTaken = 0;
volatile int windowSize = 35;
volatile double currentMean = 0;
volatile double previousMean = 0;
volatile double previousDMean = 0;
volatile double derMean = 0; 
volatile unsigned int reading = 0;
volatile int calibration = 1;
volatile int  ISRflag = 0;
volatile int heartRate = 0;
volatile int heartRateFlag = 0 ;
volatile int peakFound=0;
volatile unsigned int heartPeriod = 0;
volatile unsigned int minimumPeakPeriod=50;
volatile unsigned int babyHRGone = 0;

double stdDEV = 0;
double calibrationMean = 0;
//double NAN;

int value = LOW;
int calibrationCounter = 0;
int  calibrationSamples = 1000;
long  sum = 0;
int babyDeathPeriod = 1500;
int babyHeartAttackPeriod =2;
int ledValue = 0;
double squareMean =0;

//PIN TO ANAOLOG READ
volatile int heartPin = 0;
volatile int pulseDisplay = 9; 

// SETUP ---------------------------------------------------------------------------------------------------------------------------------------------
void setup(){
  Wire.begin();
  Wire.beginTransmission(mpu);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // Wake up the MPU-6050
  Wire.endTransmission(true);
  Serial.begin(115200);
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
  
  
  //Heart Rate Monitor Stuff
  pinMode(pulseDisplay, LOW); //HEARTRATE PIN 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  OCR1A = 125;            // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  interrupts(); 
  
}


void breathingRate(){
  if(ISRflag){
  Wire.beginTransmission(mpu);
  Wire.write(0x3B);                              // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(mpu,14,true);                // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();              // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();              // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();              // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();              // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  
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

      if(threshold - amplitude > (threshold/2)){
        if(threshold!= 0){
          //Serial.println("YOUR CHILD IS DYING!");
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
  
  
  if(changeY > 15000 && changeZ > 15000){
        fallNotice = 0;
  }

  if(changeY > 15000 && changeX > 15000){
    fallNotice = 0;
  }

  if(changeZ > 15000 && changeX > 15000){
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
}


void loop(){
  
 breathingRate();
  
//Heart Rate things
  if(calibration){
    //Serial.println("calibrating");
    if(calibrationCounter <calibrationSamples && ISRflag==1){ // going through the first 1000 samples
      Serial.println(reading);
      sum = sum + reading;
      //Serial.print("squareMean = "); Serial.println(squareMean);
      unsigned long temp = (long)reading*reading;
      //Serial.print("readinf*reading = "); Serial.println(temp);
      squareMean = ((calibrationCounter)*squareMean + temp)/(calibrationCounter + 1);
      //Serial.print("squareMean after = "); Serial.println(squareMean);
      calibrationCounter = calibrationCounter+1;
      
      ISRflag = 0; // Good to take another sample
    }
    else if(calibrationCounter == calibrationSamples){ // if we get 1000 samples
      //calculating mean
      
      calibrationMean = (double)sum/calibrationSamples;
      //calculating standard deviation 
      //Serial.print("mean of squares = "); Serial.println(squareMean);
      stdDEV = sqrt(squareMean - calibrationMean*calibrationMean);
      Serial.print("Calibration complete, mean = "); Serial.print(calibrationMean); Serial.print(" std = "); Serial.println(stdDEV);
      //set these to zero to move on or if i want to come back here again    
      calibrationCounter = 0;
      if(stdDEV != NAN){ //recalibrate if hearRate is NaN
        calibration = 0;
        heartPeriod=0;
      }
    }
  }  
  
  else{
    if(heartPeriod > babyDeathPeriod){
        //speaker shit
        Serial.println("BABY DEAD");
       // tone(buzzerPin,500);
        babyHRGone=1;
      }
    if(peakFound){
      Serial.println("DOOT doot");
      printValue("hearPeriod", heartPeriod);    
      peakFound = 0;
      heartPeriod = 0;
      
      digitalWrite(pulseDisplay, value);
      if(value == HIGH){
        value=LOW;
      }
      else if(value ==LOW){
        value=HIGH;
      }
    }
  }
  
}


void calibrateHandler(){
  //calibrateSignal = digitalRead(calibratePin);
  countMean = 0;
  //calibrate heartRate
  calibrationReset();
  
}

void calibrationReset(){
    samplesTaken = 0;
    windowSize = 35;
    currentMean = 0;
    previousMean = 0;
    previousDMean = 0;
    derMean = 0; 
    reading = 0;
    calibration = 1;
    ISRflag = 0;
    heartRate = 0;
    heartRateFlag = 0 ;
    peakFound=0;
    heartPeriod = 0;
    
    stdDEV = 0;
    calibrationMean = 0;
    value = LOW;          //value for LED
    calibrationCounter = 0;
    calibrationSamples = 1000;
    sum = 0;
    babyHeartAttackPeriod =2;
    squareMean =0;
    babyHRGone=0;
}

void printValue(String valName, double Value)
{
  Serial.print(valName); Serial.print(" = "); Serial.println(Value);
}



//
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
  //heartRate stuff
  heartPeriod=0;
  calibrationReset();
  babyHRGone=0;
}


//Heart Rate interrupt 
ISR(TIMER1_COMPA_vect){
  if (babyHRGone == 0){
  reading = analogRead(heartPin);
  ISRflag = 1;

  heartPeriod++;
  
  
  if( calibration == 0 ){
      if (samplesTaken < windowSize){ // counts up to 50, caps the samples taken at the window size
          samplesTaken = samplesTaken + 1;
      }
      //finding the current mean 
      currentMean = (previousMean*(samplesTaken-1) + reading)/samplesTaken;
      derMean = currentMean - previousMean;
      
  
      if (currentMean >= calibrationMean + stdDEV && ( previousDMean > 0 && derMean < 0 ) && heartPeriod>minimumPeakPeriod){
          //peak found
          peakFound = 1;
          //digitalWrite(pulseDisplay, HIGH);// SHOW PULSE ON LED
      }
       previousMean = currentMean;
     previousDMean = derMean;
    }
    
  }
  
}

