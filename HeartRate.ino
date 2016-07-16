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

double stdDEV = 0;
double calibrationMean = 0;
//double NAN;

int calibrationCounter = 0;
int  calibrationSamples = 1000;
long  sum = 0;
int babyDeathPeriod = 3000;
int babyHeartAttackPeriod =2;
int ledValue = 0;
double squareMean =0;

//PIN TO ANAOLOG READ
volatile int heartPin = 0;
volatile int pulseDisplay = 9; 

void setup() {
  //NAN = 1/0;
  // put your setup code here, to run once:
    pinMode(pulseDisplay, LOW); //HEARTRATE PIN
    pinMode(8,OUTPUT); //SPEAKER
    //attachInterrupt(digitalPinToInterrupt(2), initialState, CHANGE);
  //  attachInterrupt(digitalPinToInterrupt(4), resetState, CHANGE);
   // Initializes Timer2 to throw an interrupt every 2mS.
   Serial.begin(115200);
  TCCR2A = 0x02;     // DISABLE PWM ON DIGITAL PINS 3 AND 11, AND GO INTO CTC MODE
  TCCR2B = 0x06;     // DON'T FORCE COMPARE, 256 PRESCALER 
  OCR2A = 0X7C;      // SET THE TOP OF THE COUNT TO 124 FOR 500Hz SAMPLE RATE
  TIMSK2 = 0x02;     // ENABLE INTERRUPT ON MATCH BETWEEN TIMER2 AND OCR2A
  sei();             // MAKE SURE GLOBAL INTERRUPTS ARE ENABLED 
}

void printValue(String valName, double Value)
{
  Serial.print(valName); Serial.print(" = "); Serial.println(Value);
}

void loop() {

//CALIBRATION SETUP-------------------------------------------------------------------------------------------------------------------------------------------
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
      }
    }
  }else
  {

    if(peakFound){
      Serial.println("PEAK FOUND BIIITCH)");
      printValue("hearPeriod", heartPeriod);    
      ledValue ^= 1;
      digitalWrite(pulseDisplay, ledValue); 
      
      if(heartPeriod > babyDeathPeriod){
        //speaker shit
      }
      /*if(heartPeriod < babyHeartAttackPeriod){
        //speaker
      }*/
      peakFound = 0;
      heartPeriod = 0;
    }
    else{
       digitalWrite(pulseDisplay, ledValue);
      /*if(ISRflag){
        printValue("currentMean", currentMean);
        printValue("previousMean", previousMean);
        printValue("previousDMean", previousDMean);
        printValue("derMean", derMean); 
        printValue("samplesTaken", samplesTaken);
        Serial.println("----------------------------------------------------");
        ISRflag = 0;
      }*/
    }
  }
  
  
  
//END OF CALIBRATION SETUP------------------------------------------------------------------------------------------------------------------------------------
  
  
  //flag for heart rate
  /*if(heartRate <20 && heartRateFlag ==1){
      //activate buzzer
      tone(8, 1000);
      delay(1000);
      noTone(8);
      delay(1000);
  }*/
  //flag breathing
 // if(){}
  
  
  //Flag Acceleration

}

//calibration state
void initialState(){
    calibration = 1;
  
}

ISR(TIMER2_COMPA_vect){
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
      
  
      if (currentMean >= calibrationMean + stdDEV && ( previousDMean > 0 && derMean < 0 )){
          //peak found
          peakFound = 1;
          //digitalWrite(pulseDisplay, HIGH);// SHOW PULSE ON LED
      }
       previousMean = currentMean;
     previousDMean = derMean;
    }
    
    digitalWrite(pulseDisplay, LOW); //OTHERWISE KEEP IT ON THE DOWNLOW
    
     //flagStuff
   /*  if(heartRate <20){
       heartRateFlag =1;
     } */
  
  
}



/*
void resetState(){
  //reset heart rate
  heartRate = 70;
  heartRateFlag = 0;
  
  
  //Reset Breathing
  
  //Reset baby falling
  */


