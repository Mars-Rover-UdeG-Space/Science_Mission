
//This code was written for a TowerPro MG996R ~200° servo motor, and a DSSERVO DS3240MG 360°, and DS04-NFC 360°, the first one with a source voltage of 5V and 12V for the later.
#include <Arduino.h>
#define servoPin 9

float currentAngle = -1;

  unsigned long timeVariable;
  
  float lowestDutyCycle=350; //In Microseconds / 20 ms
  float higherDutyCycle=2600; //In Microseconds / 20 ms
  float maxAngle=200.00;
  int signalTime = 20000; //In Microseconds = 20ms
  unsigned long maxTime = 1050000; //Time required to rotate from 0 to maxAngle.

  String servomotor_360 = "DS3240MG";

/*void setup() {
  pinMode(servoPin, OUTPUT);
  digitalWrite(servoPin, LOW);
  
  //Serial.begin(115200);
}*/

/*void loop() {
  // put your main code here, to run repeatedly:
  if(currentAngle == -1) //Worst case scenario when starting up. Is the callibration routine
    currentAngle = maxAngle;
  
  //single_run(0);
  //single_run(200);
  //single_run(0);
  //chunk_run(10,2000*20,200);
  //chunk_run(10,2000*40,0);

//  delay(5000);
//  continuous(30,true,5.6);
//  delay(5000);
//  continuous(30,false,0.764);

//   continuous(10,true,1);
//   continuous(20,true,1);
//   continuous(30,true,1);
//   continuous(40,true,1);
//   continuous(50,true,1);
//   continuous(60,true,1);
//   continuous(70,true,1);
   continuous(60,false,20);
//   continuous(70,true,1);
//   continuous(60,true,1);
//   continuous(50,true,1);
//   continuous(40,true,1);
//   continuous(30,true,1);
//   continuous(20,true,1);
//   continuous(10,true,1);

   delay(2000);
//   
//   continuous(10,false,1);
//   continuous(20,false,1);
//   continuous(30,false,1);
//   continuous(40,false,1);
//   continuous(50,false,1);
//   continuous(60,false,1);
//   continuous(70,false,1);
//   continuous(80,false,1);
//   continuous(70,false,1);
//   continuous(60,false,1);
//   continuous(50,false,1);
//   continuous(40,false,1);
//   continuous(30,false,1);
//   continuous(20,false,1);
//   continuous(10,false,1);
}*/

void continuous(float rpm, bool direction, float time_in_seconds){ //For Continuous Rotation Servos
  //True = Clockwise, False = CounterClockwise

  int signalTime2; //In Microseconds = 30ms
  float minClockWiseRPM;
  float maxClockWiseRPM;
  int minClockWiseDutyCycle;
  int maxClockWiseDutyCycle;
  
  float minCounterClockWiseRPM;
  float maxCounterClockWiseRPM;
  int minCounterClockWiseDutyCycle;
  int maxCounterClockWiseDutyCycle;

  if(servomotor_360 == "DS3240MG"){
    signalTime2 = 30000; //In Microseconds = 30ms
    minClockWiseRPM=7;
    maxClockWiseRPM=88;
    minClockWiseDutyCycle=1453;
    maxClockWiseDutyCycle=1390;
  
    minCounterClockWiseRPM=10.7;
    maxCounterClockWiseRPM=68;
    minCounterClockWiseDutyCycle=1460;
    maxCounterClockWiseDutyCycle=1525;
  }
  else if(servomotor_360 == "DS04-NFC"){
    signalTime2 = 20000; //In Microseconds = 30ms
    minClockWiseRPM=3.45;
    maxClockWiseRPM=50;
    minClockWiseDutyCycle=1431;
    maxClockWiseDutyCycle=1050;
  
    minCounterClockWiseRPM=3;
    maxCounterClockWiseRPM=50;
    minCounterClockWiseDutyCycle=1543;
    maxCounterClockWiseDutyCycle=1980;
  }

  //Serial.println("");
  //Serial.print("Rotating at : ");//Serial.print(rpm);//Serial.print(" RPM. ");
  if(direction){
    //Serial.println("Clockwise direction.");
  }else{
    //Serial.println("Counter clockwise direction.");
  }

  if( (direction && rpm <= maxClockWiseRPM && rpm >= minClockWiseRPM) || (!direction && rpm <= maxCounterClockWiseRPM && rpm >= minCounterClockWiseRPM) ){

    unsigned long rotationTimeMicroseconds = time_in_seconds * 1000000;
    float DutyCyclePerRPM;
    int timeOn;
    int timeOff;
      
    if(direction){ //Clockwise
      DutyCyclePerRPM = (minClockWiseDutyCycle - maxClockWiseDutyCycle)/(maxClockWiseRPM - minClockWiseRPM);
      timeOn = minClockWiseDutyCycle - round(DutyCyclePerRPM * rpm);
      timeOff = signalTime2 - timeOn;
    }
    else if(!direction){ //Counter Clockwise
      DutyCyclePerRPM = (maxCounterClockWiseDutyCycle - minCounterClockWiseDutyCycle)/(maxCounterClockWiseRPM - minCounterClockWiseRPM);
      timeOn = minCounterClockWiseDutyCycle + round(DutyCyclePerRPM * rpm);
      timeOff = signalTime2 - timeOn;
    }

    timeVariable = micros();
    while(true){
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(timeOn);
      digitalWrite(servoPin, LOW);
      delayMicroseconds(timeOff);
      if( (micros() - timeVariable) > rotationTimeMicroseconds ){
        //Serial.println( (micros() - timeVariable) );
        break;
      }
    }
  }
  else{
    //Serial.println("RPM speed out of boundaries for selected direction");
  }
}

void single_run(float angle){ //For Arc Rotation Servos

  //Serial.println("");
  //Serial.print("Single run. From : ");//Serial.print(currentAngle);//Serial.print(" to: ");//Serial.println(angle);

  if(angle <= maxAngle && angle >= 0){
    int timeOn = lowestDutyCycle + round(((higherDutyCycle-lowestDutyCycle)/maxAngle) * angle);
    int timeOff = signalTime - timeOn;
    //Serial.print("ON Time: "); //Serial.println(timeOn);
    //Serial.print("OFF Time: "); //Serial.println(timeOff);
  
    float arcAngle = abs(currentAngle - angle);
    unsigned long waitTime = (1050000/maxAngle)*arcAngle;
    //Serial.print("Wait Time: "); //Serial.println(waitTime);
    
    timeVariable = micros();
    while(true){
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(timeOn);
      digitalWrite(servoPin, LOW);
      delayMicroseconds(timeOff);
      if( (micros() - timeVariable) > waitTime ){
        ////Serial.println( (micros() - timeVariable) );
        break;
      }
    }
    //Serial.print("Total time: "); //Serial.println( (micros() - timeVariable) );
    currentAngle = angle; //Updates the current angle
  }
  else{
    //Serial.println("Angle out of boundaries");
  }
}

void chunk_run(int chunk_width_us,int delay_us, float angle){ //For Arc Rotation Servos

  //Serial.println("");
  //Serial.print("Chuck run. From : ");//Serial.print(currentAngle);//Serial.print(" to: ");//Serial.println(angle);
  
  if( (angle <= maxAngle && angle >= 0) && (chunk_width_us <= 12 && chunk_width_us >= 1) ){
    float targetArcAngle = abs(currentAngle - angle);
    float anglePerChunkWidth = (maxAngle/(higherDutyCycle-lowestDutyCycle)) * chunk_width_us;
    int nChunks = floor(targetArcAngle/anglePerChunkWidth);
    int timeOnArray[nChunks];
    int timeOffArray[nChunks];
    int waitTime;

    //Serial.print("anglePerChunk: "); //Serial.println(anglePerChunkWidth);
    //Serial.print("nChunks: "); //Serial.println(nChunks);

    float nextAngle=currentAngle;
    if(currentAngle > angle){ //Inverts the anglePerChunkWidth when the angle will decrement.
      anglePerChunkWidth*= -1;
    }
    
    for(int chunk=0 ; chunk<nChunks ; chunk++){
      if( chunk==(nChunks-1) ){
        timeOnArray[chunk] = lowestDutyCycle + round(((higherDutyCycle-lowestDutyCycle)/maxAngle) * angle);
        timeOffArray[chunk] = signalTime - timeOnArray[chunk];
      }
      else{
        timeOnArray[chunk] = lowestDutyCycle + round(((higherDutyCycle-lowestDutyCycle)/maxAngle) * nextAngle);
        timeOffArray[chunk] = signalTime - timeOnArray[chunk];
        ////Serial.print("ON Time: "); //Serial.println(timeOnArray[chunk]);
        ////Serial.print("OFF Time: "); //Serial.println(timeOffArray[chunk]);
      }
      nextAngle+=anglePerChunkWidth;
    }
  
    float arcAngle = abs(currentAngle - anglePerChunkWidth);
    waitTime = (1050000/maxAngle)*arcAngle;
    ////Serial.print("Wait Time: "); //Serial.println(waitTime);

    timeVariable = micros();
    for(int n=0 ; n<nChunks ; n++){
      digitalWrite(servoPin, HIGH);
      delayMicroseconds(timeOnArray[n]);
      digitalWrite(servoPin, LOW);
      delayMicroseconds(timeOffArray[n]);
      if( (micros() - timeVariable) > waitTime ){
        ////Serial.println( (micros() - timeVariable) );
        delayMicroseconds(delay_us); //Delay in microseconds
        continue;
      }
    }
    //Serial.print("Total time: "); //Serial.println( (micros() - timeVariable) );
    
    currentAngle = angle; //Updates the current angle
  }
  else{
    //Serial.println("Angle or chunk width out of boundaries");
  }
}