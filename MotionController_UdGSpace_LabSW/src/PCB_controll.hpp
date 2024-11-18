// Author: Jesus Delgadillo
// Version: PCBPinExtensions VT0.01
// Notes: This code library is designed for controlling the logical electronic structure controlling the motors for the UdeG MARS Rover. This is a test code, not dynamic.

// Arduino Functions
// pinMode(PIN, OUTPUT) 
// digitalWrite(PIN, VALUE)
// Variable = digitalRead(PIN)
// Variable = analogRead(PIN)


/*

    extended_pinMode(); 
    extended_digitalWrite();    
    extended_digitalRead(); 

*/

#include <Arduino.h>

#include <iostream>
#include <string>
#include <vector>
#include <bitset>
#include <Wire.h> //I2C Support
#include <SPI.h> //SPI Support


#define DEMUX "DEMUX"
#define DIOE "DIOE"
#define INHE "Inherited"


class DMUX;
class IOE;
class ExtendedPin;

ExtendedPin* pinLocator(int);

void extended_pinMode(int pinNumber, int pinmode);
void extended_digitalWrite(int pinNumber, bool wvalue);
int extended_digitalRead(int pinNumber);

int binaryvectorto_int(std::vector<int>* binaryVector, int start_bit=-1, int end_bit=-1, bool invert_vector = true){
    int binaryLength;
    
    if(start_bit == -1 && end_bit == -1){ //Converts the entire vector by default.
        start_bit = 0;
        end_bit = binaryVector->size()-1;
        binaryLength = binaryVector->size();
    }
    
    binaryLength = end_bit + 1 - start_bit;
    
    int integer_value = 0;

    if(invert_vector == false){
      for(int i=start_bit, bitcount = 0, bitexponent = binaryLength-1 ; bitcount<binaryLength ; i++, bitcount++, bitexponent-- ){
          int bitvalue =  pow(2,bitexponent);
          //////Serial.print("Iterating position: "); ////Serial.print(i); ////Serial.print(" With value: "); ////Serial.print(binaryVector->at(i)); ////Serial.print(" bitvalue: "); ////Serial.println(bitvalue);
          if(binaryVector->at(i) == 1){ //if the bit equals 1
              integer_value += bitvalue;
          }
      }
    }
    else if(invert_vector == true){
      for(int i=end_bit, bitcount = 0, bitexponent = binaryLength-1 ; bitcount<binaryLength ; i--, bitcount++, bitexponent-- ){
          int bitvalue =  pow(2,bitexponent);
          //////Serial.print("Iterating position: "); ////Serial.print(i); ////Serial.print(" With value: "); ////Serial.print(binaryVector->at(i)); ////Serial.print(" bitvalue: "); ////Serial.println(bitvalue);
          if(binaryVector->at(i) == 1){ //if the bit equals 1
              integer_value += bitvalue;
          }
      }
    }

    //////Serial.print("Integer: "); ////Serial.println(integer_value);
    return integer_value;
}

int binaryvectorto_int(std::vector<bool>* binaryVector, int start_bit=-1, int end_bit=-1, bool invert_vector = true){
    int binaryLength;
    
    if(start_bit == -1 && end_bit == -1){ //Converts the entire vector by default.
        start_bit = 0;
        end_bit = binaryVector->size()-1;
        binaryLength = binaryVector->size();
    }
    
    binaryLength = end_bit + 1 - start_bit;
    
    int integer_value = 0;

    if(invert_vector == false){
      for(int i=start_bit, bitcount = 0, bitexponent = binaryLength-1 ; bitcount<binaryLength ; i++, bitcount++, bitexponent-- ){
          int bitvalue =  pow(2,bitexponent);
          //////Serial.print("Iterating position: "); ////Serial.print(i); ////Serial.print(" With value: "); ////Serial.print(binaryVector->at(i)); ////Serial.print(" bitvalue: "); ////Serial.println(bitvalue);
          if(binaryVector->at(i) == true){ //if the bit equals 1
              integer_value += bitvalue;
          }
      }
    }
    else if(invert_vector == true){
      for(int i=end_bit, bitcount = 0, bitexponent = binaryLength-1 ; bitcount<binaryLength ; i--, bitcount++, bitexponent-- ){
          int bitvalue =  pow(2,bitexponent);
          //////Serial.print("Iterating position: "); ////Serial.print(i); ////Serial.print(" With value: "); ////Serial.print(binaryVector->at(i)); ////Serial.print(" bitvalue: "); ////Serial.println(bitvalue);
          if(binaryVector->at(i) == 1){ //if the bit equals 1
              integer_value += bitvalue;
          }
      }
    }

    //////Serial.print("Integer: "); ////Serial.println(integer_value);
    return integer_value;
}

//---------------------- De/Multiplexers ----------------------

class DMUX{
private:
    
public:
    std::string identifier;
    
    int* pinInputSignal1;
    int* pinInputSignal2;
    int* pinChannelSelector0;
    int* pinChannelSelector1;
    int* pinChannelSelector2;
    int* pinChannelSelector3;

    int channelIOs; //Number of channels
    int lineIOs;    //Number of lines per channel
    int channelSelectors;
    int pinEnable;
    
    int DeMuxIOs;   //Total of pins
    
    std::vector<int> ioModeVector;  //Empty vector //3 = output pin, 1 = input pin // Channels are Input pins by default
    std::vector<bool> stateVector;   //Empty vector //Represents the digital value stored (received or sent) for each gpio.
    int selectedChannel;
    
    bool channelvalidator(int); //true/1 = valid pin, false/0 = invalid pin
    bool pinvalidator(int);  
    void routechannel(int);
    void setmodepin(int,int);
    int readchannel(int, int);
    int readpin(int);
    bool writechannel(int,int,bool);
    void writepin(int,bool);
    void enable();
    void disable();
    
};

bool DMUX::pinvalidator(int pin){
    ////Serial.print("Calling DMUX pinvalidator");  ////Serial.print(" - "); ////Serial.println(pin);

    if(pin > DeMuxIOs){ //Invalid channel for this DeMuxer
        return false;
    }
    return true; //Valid channel
}

bool DMUX::channelvalidator(int schannel){
    ////Serial.print("Calling DMUX channelvalidator"); ////Serial.print(" - "); ////Serial.println(schannel);

    if(schannel > channelIOs){ //Invalid channel for this DeMuxer
        ////Serial.println("\t - Channel is invalid.");
        return false;
    }
    ////Serial.println("\t - Channel is valid.");
    return true; //Valid channel
}

void DMUX::routechannel(int schannel){
    ////Serial.print("Calling DMUX routechannel");  ////Serial.print(" - "); ////Serial.println(schannel);

    if(!channelvalidator(schannel)){
        return;
    }

    if(digitalRead(pinEnable) == HIGH){ //Enables the channel if the DeMuxer is disabled
        enable();
    }

    if(schannel == selectedChannel){// Route the channel only if the selectedChannel differs.
        return;
    }

    //These conditionals place the pins as high impedance input pins, before routing the channel.
    if(lineIOs >= 1){
      pinMode( *pinInputSignal1, INPUT );
    }
    if(lineIOs >= 2){
      pinMode( *pinInputSignal2, INPUT );
    }
    
    selectedChannel = schannel; //Updates the selected channel
    
    std::bitset<4> binarychannel(schannel); //Converts an integer to binary representation. 4 Represents the max binary digits that can be represented by the variable.
    
    if(channelSelectors >= 2){
        digitalWrite( *pinChannelSelector0, int(binarychannel[0]) );
        ////Serial.print("\t - Modifying inherited pin selector "); ////Serial.println(*pinChannelSelector0);
        digitalWrite( *pinChannelSelector1, int(binarychannel[1]) );
        ////Serial.print("\t - Modifying inherited pin selector"); ////Serial.println(*pinChannelSelector1);
    }
    if(channelSelectors >= 3){
        digitalWrite( *pinChannelSelector2, int(binarychannel[2]) );
        ////Serial.print("\t - Modifying inherited pin selector"); //Serial.println(*pinChannelSelector2);
    }
    if(channelSelectors >= 4){
        digitalWrite( *pinChannelSelector3, int(binarychannel[3]) );
        //Serial.print("\t - Modifying inherited pin selector"); //Serial.println(*pinChannelSelector3);
    }

    //These conditionals place the pins as determined by the ioModeVector, after routing the channel.
    if(lineIOs >= 1){
      pinMode( *pinInputSignal1, ioModeVector[schannel*lineIOs] );
    }
    if(lineIOs >= 2){
      pinMode( *pinInputSignal2, ioModeVector[schannel*lineIOs+1] );
    }

}

void DMUX::setmodepin(int pin, int pinmode){
    //Serial.println("Calling DMUX setmodepin");

    if(!pinvalidator(pin)){
        return;
    }

    ioModeVector[pin] = pinmode; //Sets only the state in the ioModeVector.
}

int DMUX::readpin(int pin){ 
    //Serial.print("Calling DMUX readpin");  //Serial.print(" - "); //Serial.println(pin);

    if(pin > channelIOs-1){ //The DeMux has 2 lines per channel
      int schannel = ceil(float(pin)/2.0);
      return readchannel(schannel,2);
    }
    else{ //The DeMux has 1 lines per channel
      return readchannel(pin,1);
    }
}

int DMUX::readchannel(int schannel, int line){
    //Serial.print("Calling DMUX readchannel");  //Serial.print(" - "); //Serial.print(schannel); //Serial.print(",");//Serial.println(line);

    //Both output pins and input pins can be read
    int readvalue;
    if(!channelvalidator(schannel)){
        return -1;
    }

    routechannel(schannel);

    if(pinInputSignal1 != nullptr && line == 1){
        stateVector[schannel*lineIOs]=digitalRead(*pinInputSignal1);
    }
    if(pinInputSignal2 != nullptr && line == 2){
        stateVector[schannel*lineIOs+1]=digitalRead(*pinInputSignal2);
    }

    readvalue = stateVector[schannel*lineIOs+line];

    return(readvalue);
}

void DMUX::writepin(int pin,bool digitalSignal){
    //Serial.print("Calling DMUX writepin");  //Serial.print(" - "); //Serial.println(pin);

    int line;

    if(pin > channelIOs-1){ //The DeMux has 2 lines per channel
      int schannel = ceil(float(pin)/2.0);
      writechannel(schannel,2,digitalSignal);
    }
    else{ //The DeMux has 1 lines per channel
      writechannel(pin,1,digitalSignal);
    }
}

bool DMUX::writechannel(int schannel, int line, bool digitalSignal){
    //Serial.print("Calling DMUX writechannel"); //Serial.print(" - "); //Serial.print(schannel); //Serial.print(",");//Serial.println(line);

    //Only output pins can be writen.
    if(!channelvalidator(schannel)){
        return false;
    }

    routechannel(schannel);

    if(pinInputSignal1 != nullptr && line == 1){
        digitalWrite(*pinInputSignal1,digitalSignal);
        //Serial.print("\t - Modifying inherited pin "); //Serial.println(*pinInputSignal1);
        //Serial.print("\t - Writing value "); //Serial.println(digitalSignal);
        stateVector[schannel*lineIOs+line]=digitalSignal;
        return true;
    }
    if(pinInputSignal2 != nullptr && line == 2){
        digitalWrite(*pinInputSignal2,digitalSignal);
        //Serial.print("\t - Modifying inherited pin "); //Serial.println(*pinInputSignal1);
        //Serial.print("\t - Writing value "); //Serial.println(digitalSignal);
        stateVector[schannel*lineIOs+line]=digitalSignal;
        return true;
    }

    return false; //Error
}

void DMUX::enable(){
    //Serial.println("Calling DMUX enable");

    if( digitalRead(pinEnable) == LOW ){
        return; //DeMuxer is already enabled
    }
    digitalWrite(pinEnable, LOW);
    //Serial.print("\t - Modifying enable pin "); //Serial.println(pinEnable);
}

void DMUX::disable(){
    //Serial.println("Calling DMUX disable");

    if( digitalRead(pinEnable) == HIGH ){
        return; //DeMuxer is already disabled
    }
    digitalWrite(pinEnable, HIGH);
    //Serial.print("\t - Modifying enable pin "); //Serial.println(pinEnable);
}

class IC_74HC4067 : public DMUX{
private:

public:
    IC_74HC4067(std::string,int,int,int,int,int,int);

};

IC_74HC4067::IC_74HC4067(std::string identifier,int pinInputSignal1,int pinChannelSelector3,int pinChannelSelector2,int pinChannelSelector1,int pinChannelSelector0,int pinEnable){
    //Serial.println("Setting a 74HC4067 DeMuxer");

    this->identifier = identifier;

    this->pinInputSignal1 = new int; *this->pinInputSignal1 = pinInputSignal1;
    this->pinChannelSelector0 = new int; *this->pinChannelSelector0 = pinChannelSelector0;
    this->pinChannelSelector1 = new int; *this->pinChannelSelector1 = pinChannelSelector1;
    this->pinChannelSelector2 = new int; *this->pinChannelSelector2 = pinChannelSelector2;
    this->pinChannelSelector3 = new int; *this->pinChannelSelector3 = pinChannelSelector3;
    this->pinEnable = pinEnable;

    channelIOs = 16; //Number of channels
    lineIOs = 1;    //Number of lines per channel
    channelSelectors = 4;

    DeMuxIOs = channelIOs*lineIOs;   //Total of pins

    //Pin settings
    pinMode(*this->pinInputSignal1,OUTPUT); digitalWrite(*this->pinInputSignal1,LOW);
    pinMode(*this->pinChannelSelector0,OUTPUT); digitalWrite(*this->pinChannelSelector0,LOW);
    pinMode(*this->pinChannelSelector1,OUTPUT); digitalWrite(*this->pinChannelSelector1,LOW);
    pinMode(*this->pinChannelSelector2,OUTPUT); digitalWrite(*this->pinChannelSelector2,LOW);
    pinMode(*this->pinChannelSelector3,OUTPUT); digitalWrite(*this->pinChannelSelector2,LOW);
    pinMode(pinEnable,OUTPUT); digitalWrite(pinEnable,HIGH); //Initializes the DeMuxer disabled.

    ioModeVector.resize(16);      //8-bit vector //0 = output pin, 1 = input pin // Channels are Input pins by default
    ioModeVector.assign(16,1);   //Initializes all content assigned to 1. Signaling the pins are inputs.
    stateVector.resize(16);      //8-bit vector //Represents the digital value stored (received or sent) for each gpio.
    stateVector.assign(16,0);    //Initializes all content assigned to 0. Signaling the pins as outputs must output a LOW value.
    selectedChannel = 0; //Line 0 is enabled initially.
}

class IC_74HC4051 : public DMUX{
private:
    
public:
    IC_74HC4051(std::string,int,int,int,int,int);
    
};

IC_74HC4051::IC_74HC4051(std::string identifier,int pinInputSignal1,int pinChannelSelector2,int pinChannelSelector1,int pinChannelSelector0,int pinEnable){
    //Serial.println("Setting a 74HC4051 DeMuxer");

    this->identifier = identifier;

    this->pinInputSignal1 = new int; *this->pinInputSignal1 = pinInputSignal1;
    this->pinChannelSelector0 = new int; *this->pinChannelSelector0 = pinChannelSelector0;
    this->pinChannelSelector1 = new int; *this->pinChannelSelector1 = pinChannelSelector1;
    this->pinChannelSelector2 = new int; *this->pinChannelSelector2 = pinChannelSelector2;
    this->pinEnable = pinEnable;

    channelIOs = 8; //Number of channels
    lineIOs = 1;    //Number of lines per channel
    channelSelectors = 3;

    DeMuxIOs = channelIOs*lineIOs;   //Total of pins

    //Pin settings
    pinMode(*this->pinInputSignal1,OUTPUT); digitalWrite(*this->pinInputSignal1,LOW);
    pinMode(*this->pinChannelSelector0,OUTPUT); digitalWrite(*this->pinChannelSelector0,LOW);
    pinMode(*this->pinChannelSelector1,OUTPUT); digitalWrite(*this->pinChannelSelector1,LOW);
    pinMode(*this->pinChannelSelector2,OUTPUT); digitalWrite(*this->pinChannelSelector2,LOW);
    pinMode(pinEnable,OUTPUT); digitalWrite(pinEnable,HIGH); //Initializes the DeMuxer disabled.

    ioModeVector.resize(8);      //8-bit vector //0 = output pin, 1 = input pin // Channels are Input pins by default
    ioModeVector.assign(8,1);   //Initializes all content assigned to 1. Signaling the pins are inputs.
    stateVector.resize(8);      //8-bit vector //Represents the digital value stored (received or sent) for each gpio.
    stateVector.assign(8,0);    //Initializes all content assigned to 0. Signaling the pins as outputs must output a LOW value.
    selectedChannel = 0; //Line 0 is enabled initially.
}

class IC_74HC4052 : public DMUX{
private:
    
public:
    IC_74HC4052(std::string,int,int,int,int,int);
};

IC_74HC4052::IC_74HC4052(std::string identifier,int pinInputSignal2,int pinInputSignal1,int pinChannelSelector2,int pinChannelSelector1,int pinEnable){
    //Serial.println("Setting a 74HC4052 DeMuxer");

    this->identifier = identifier;

    this->pinInputSignal1 = new int; *this->pinInputSignal1 = pinInputSignal1;
    this->pinInputSignal2 = new int; *this->pinInputSignal2 = pinInputSignal2;
    this->pinChannelSelector0 = new int;
    this->pinChannelSelector0 = pinChannelSelector0;
    this->pinChannelSelector1 = new int; *this->pinChannelSelector1 = pinChannelSelector1;
    this->pinEnable = pinEnable;

    channelIOs = 4; //Number of channels
    lineIOs = 2;    //Number of lines per channel
    channelSelectors = 2;

    DeMuxIOs = channelIOs*lineIOs;   //Total of pins

    //Pin settings
    pinMode(*this->pinInputSignal1,OUTPUT); digitalWrite(*this->pinInputSignal1,LOW);
    pinMode(*this->pinInputSignal2,OUTPUT); digitalWrite(*this->pinInputSignal2,LOW);
    pinMode(*this->pinChannelSelector0,OUTPUT); digitalWrite(*this->pinChannelSelector0,LOW);
    pinMode(*this->pinChannelSelector1,OUTPUT); digitalWrite(*this->pinChannelSelector1,LOW);
    pinMode(pinEnable,OUTPUT); digitalWrite(pinEnable,HIGH); //Initializes the DeMuxer disabled.

    ioModeVector.resize(8);      //8-bit vector //0 = output pin, 1 = input pin // Channels are Input pins by default
    ioModeVector.assign(8,1);   //Initializes all content assigned to 1. Signaling the pins are inputs.
    stateVector.resize(8);      //8-bit vector //Represents the digital value stored (received or sent) for each gpio.
    stateVector.assign(8,0);    //Initializes all content assigned to 0. Signaling the pins as outputs must output a LOW value.
    selectedChannel = 0; //Line 0 is enabled initially.
}

//---------------------- IO Expanders ----------------------

class IOE{
private:
    
public:
    std::string identifier;
    
    int GPIOs; //Total number of gpios
    int pinSCL;
    int pinSDA;
    
    int pinINT;
    int pinRST;
    
    //IC Properties
    //--- Semi-dynamic variables
    std::bitset<7> I2C_Address; // Stores the 7-bit address
    
    //--- Dynamic variables
    std::vector<int> ioModeVector; //Empty vector //0 = output pin, 1 = input pin. Sequential from pin 0 to 15
    std::vector<bool> stateVector;   //Empty vector //Represents the digital value stored (received or sent) for each gpio. Sequential from pin 0 to 15
    
    //--- Static variables
    std::bitset<8>* inputPortsRegister;
    std::bitset<8> inputPortsRegister0; //Empty register
    std::bitset<8> inputPortsRegister1; //Empty register
    std::bitset<8>* outputPortsRegister;
    std::bitset<8> outputPortsRegister0; //Empty register
    std::bitset<8> outputPortsRegister1; //Empty register
    std::bitset<8>* polarityRegister;
    std::bitset<8> polarityRegister0; //Empty register
    std::bitset<8> polarityRegister1; //Empty register
    std::bitset<8>* configurationRegister;
    std::bitset<8> configurationRegister0; //Empty register
    std::bitset<8> configurationRegister1; //Empty register
    
    //Methods
    bool pinvalidator(int); //true/1 = valid pin, false/0 = invalid pin
    void registerselector(int);
    void setmodegpio(int,int);
    int readgpio(int);
    void writegpio(int,bool);
    void reset();
};

bool IOE::pinvalidator(int gpio){
    //Serial.print("Calling IOE pinvalidator");  //Serial.print(" - "); //Serial.println(gpio);
  
    if(gpio > GPIOs){ //Invalid pin for this IO Expander
        return false;
    }
    return true; //Valid channel
}
void IOE::registerselector(int gpio){
    //Serial.print("Calling IOE registerselector");  //Serial.print(" - "); //Serial.println(gpio);
  
    if(gpio <= 7){
        //Selects the registers 0
        inputPortsRegister = &inputPortsRegister0;
        outputPortsRegister = &outputPortsRegister0;
        polarityRegister = &polarityRegister0;
        configurationRegister = &configurationRegister0;
    }
    else if(gpio > 7){
        //Selects the registers 1
        inputPortsRegister = &inputPortsRegister1;
        outputPortsRegister = &outputPortsRegister1;
        polarityRegister = &polarityRegister1;
        configurationRegister = &configurationRegister1;
    }
}
void IOE::setmodegpio(int gpio, int pinmode){
    //Serial.print("Calling IOE setmodegpio");  //Serial.print(" - "); //Serial.print(gpio); //Serial.print(","); //Serial.println(pinmode);
  
    if(!pinvalidator(gpio)){
        return;
    }

    if(pinmode == 3){// Translates the arduino default INPUT, OUTPUT (Output: 3, Input: 1) default modes to valid modes for the IC (Output: 0, Input: 1)
      pinmode = 0;
    }
    
    registerselector(gpio);

    Wire.beginTransmission( int(I2C_Address.to_ulong()) ); //Begins transmission to IOE IC. 
    // The R/W set bit is trandsmitted according to write or read methods.
    //Serial.print("\t I2C address: "); //Serial.print( int(I2C_Address.to_ulong()) ); //Serial.print(" ");
    Wire.write( int(configurationRegister->to_ulong()) );  //Queues the command byte
    //Serial.print(", Command byte: "); //Serial.print( int(configurationRegister->to_ulong()) ); //Serial.print(" ");

    ioModeVector[gpio] = pinmode; //Modifies the mode to be sent over to the IOE.

    //Serial.print(", Data byte: ");
    if( ioModeVector.size() == 8 ){ //It's an 8th-bit IOE.
        Wire.write( binaryvectorto_int(&ioModeVector) );  //Queues the data byte
        //Serial.print( binaryvectorto_int(&ioModeVector) ); //Serial.println(" ");
    }
    else if( ioModeVector.size() == 16 ){ //It's an 16th-bit IOE.
        if(gpio <= 7){
            Wire.write( binaryvectorto_int(&ioModeVector,0,7) ); //Queues the data byte
            //Serial.print( binaryvectorto_int(&ioModeVector,0,7) ); //Serial.println(" ");
        }
        else if(gpio > 7){
            Wire.write( binaryvectorto_int(&ioModeVector,8,15) ); //Queues the data byte
            //Serial.print( binaryvectorto_int(&ioModeVector,8,15) ); //Serial.println(" ");
        }
    }
    
    if( Wire.endTransmission(true) != 0 ){ //Ends transmission sending the queued bytes. Last transmitted byte.
        return; //failure. Received NACK or other error.
    }
    delay(5);
}
int IOE::readgpio(int gpio){
    //Serial.print("Calling IOE readgpio");  //Serial.print(" - "); //Serial.println(gpio);
  
    int readvalue;
  
    if(!pinvalidator(gpio)){
        return -1;
    }

    registerselector(gpio);

    Wire.beginTransmission( int(I2C_Address.to_ulong()) ); //Begins transmission to IOE IC. Transmits the address.
    // The R/W set bit is trandsmitted according to write or read methods.
    //Serial.print("\t I2C address: "); //Serial.print( int(I2C_Address.to_ulong()) ); //Serial.print(" ");
    Wire.write( int(inputPortsRegister->to_ulong()) );  //Queues the command byte
    //Serial.print(", Command byte: "); //Serial.print( int(inputPortsRegister->to_ulong()) ); //Serial.print(" ");
    if( Wire.endTransmission(true) != 0 ){ //Ends transmission sending the queued bytes.
        return -1; //failure. Received NACK or other error.
    }

    Wire.requestFrom( int(I2C_Address.to_ulong()) ,1, true); //Begins request from IOE IC. Transmits the address for request.
    // The R/W set bit is transmitted according to write or read methods.
    while(Wire.available()) {
        readvalue = Wire.read(); //MISSING CODE. FIX THIS
    }

    return readvalue;
}
void IOE::writegpio(int gpio,bool value){
    //Serial.print("Calling IOE writegpio");  //Serial.print(" - "); //Serial.println(gpio);
  
    if(!pinvalidator(gpio)){
        return;
    }

    registerselector(gpio);

    Wire.beginTransmission( int(I2C_Address.to_ulong()) ); //Begins transmission to IOE IC. Transmits the address.
    // The R/W set bit is trandsmitted according to write or read methods.
    //Serial.print("\t I2C address: "); //Serial.print( int(I2C_Address.to_ulong()) ); //Serial.print(" ");
    Wire.write( int(outputPortsRegister->to_ulong()) );  //Queues the command byte
    ////Serial.print(", Command byte: "); ////Serial.print( int(outputPortsRegister->to_ulong()) ); ////Serial.print(" ");

    //////Serial.print(stateVector[gpio]); ////Serial.print(", "); ////Serial.println(value);
    stateVector[gpio] = value; //Modifies the mode to be sent over to the IOE.
    //////Serial.print(stateVector[gpio]); ////Serial.print(", "); ////Serial.println(value);

    ////Serial.print(", Data byte: ");
    if( stateVector.size() == 8 ){ //It's an 8th-bit IOE.
        Wire.write( binaryvectorto_int(&stateVector) );  //Queues the data byte
        ////Serial.print( binaryvectorto_int(&stateVector) ); ////Serial.println(" ");
    }
    else if( stateVector.size() == 16 ){ //It's an 16th-bit IOE.
        if(gpio <= 7){
            Wire.write( binaryvectorto_int(&stateVector,0,7) ); //Queues the data byte
            ////Serial.print( binaryvectorto_int(&stateVector,0,7) ); ////Serial.println(" ");
        }
        else if(gpio > 7){
            Wire.write( binaryvectorto_int(&stateVector,8,15) ); //Queues the data byte
            ////Serial.print( binaryvectorto_int(&stateVector,8,15) ); ////Serial.println(" ");
        }
    }

    if( Wire.endTransmission(true) != 0 ){ //Ends transmission sending the queued bytes. Last transmitted byte.
        return; //failure. Received NACK or other error.
    }
    delay(5);
}
/*void reset(){
    
}/*/

class IC_PCA9555 : public IOE{
private:

public:
    IC_PCA9555(std::string,std::bitset<7>,int,int,int,int);
};

IC_PCA9555::IC_PCA9555(std::string identifier,std::bitset<7> I2C_Address,int pinSCL,int pinSDA,int pinINT,int pinRST){
    this->identifier = identifier;
    this->I2C_Address = I2C_Address;
    //this->I2C_Address = int(I2C_Address.to_ulong());
    //this->I2C_Address = this->I2C_Address << 1; //Stores the I2C address and performs one position binary shift to the left.

    this->pinSCL = pinSCL;
    this->pinSDA = pinSDA;

    this->pinINT = pinINT;
    this->pinRST = pinRST;

    this->GPIOs = 16;

    //Pin settings
    pinMode(pinINT,INPUT);
    pinMode(pinRST,OUTPUT); digitalWrite(pinRST,HIGH); //Initializes the IOE enabled.

    ioModeVector.resize(16);      //16-bit vector //0 = output pin, 1 = input pin // Channels are Input pins by default
    ioModeVector.assign(16,1);   //Initializes all content assigned to 1. Signaling the pins are inputs. Which are the Default values for Configuration ports 1 and 0
    stateVector.resize(16);      //16-bit vector //Represents the digital value stored (received or sent) for each GPIO.
    stateVector.assign(16,1);    //Initializes all content assigned to 0. Signaling the pins as outputs must output a HIGH value.Which are the Default values for Output ports 1 and 0

    //--- Defined registers
    inputPortsRegister0 = 0b00000000;
    inputPortsRegister1 = 0b00000001;
    outputPortsRegister0 = 0b00000010;
    outputPortsRegister1 = 0b00000011;
    polarityRegister0 = 0b00000100;
    polarityRegister1 = 0b00000101;
    configurationRegister0 = 0b00000110;
    configurationRegister1 = 0b00000111;
}

class IC_PCAL9538A : public IOE{
private:

public:
    IC_PCAL9538A(std::string,std::bitset<7>,int,int,int,int);
};

IC_PCAL9538A::IC_PCAL9538A(std::string identifier,std::bitset<7> I2C_Address,int pinSCL,int pinSDA,int pinINT,int pinRST){
    this->identifier = identifier;
    this->I2C_Address = I2C_Address;
    //this->I2C_Address = int(I2C_Address.to_ulong());
    //this->I2C_Address = this->I2C_Address << 1; //Stores the I2C address and performs one position binary shift to the left.

    this->pinSCL = pinSCL;
    this->pinSDA = pinSDA;

    this->pinINT = pinINT;
    this->pinRST = pinRST;

    this->GPIOs = 8;

    //Pin settings
    pinMode(pinINT,INPUT);
    pinMode(pinRST,OUTPUT); digitalWrite(pinRST,HIGH); //Initializes the IOE enabled.

    ioModeVector.resize(8);      //8-bit vector //0 = output pin, 1 = input pin // Channels are Input pins by default
    ioModeVector.assign(8,1);   //Initializes all content assigned to 1. Signaling the pins are inputs. Which are the Default values for Configuration ports 1 and 0
    stateVector.resize(8);      //8-bit vector //Represents the digital value stored (received or sent) for each GPIO.
    stateVector.assign(8,1);    //Initializes all content assigned to 0. Signaling the pins as outputs must output a HIGH value.Which are the Default values for Output ports 1 and 0

    //--- Defined registers
    inputPortsRegister0 = 0b00000000;
    outputPortsRegister0 = 0b00000001;
    polarityRegister0 = 0b00000010;
    configurationRegister0 = 0b00000011;
}

//---------------------- ADC ----------------------

/*
class ADC{
private:
    
public:
    std::string identifier;
    
    std::string SPI_mode;
    
    int GPIOs;
    int AINs;
    int bitResolution;
    
    int pinCS;
    int pinSCLK;
    int pinSDO;
    int pinSDI;
    
};

class IC_ADS7066 : public ADC{
private:
    int ioModeTable[8] = {2,2,2,2,2,2,2,2}; //0 = digital read, 1 = digital write, 2 = analog sampling
    bool stateTable[8] = {0,0,0,0,0,0,0,0};
public:
    
};
*/

class ExtendedPin{
private:
    std::string type=""; //Available types: "DEMUX", "DIOE", "AIOE", "Inherited"
    int pinNumber;
    std::string identifier="";
    
    void* parentIC;
    int parentIC_PortNumber;
public:
    ExtendedPin(const char *,int,const char *,void*,int);

    void pinset(int);
    int pinread();
    void pinwrite(bool);  
};

ExtendedPin::ExtendedPin(const char * identifier, int pinNumber, const char * type, void* parentIC, int parentIC_PortNumber){
    this->identifier = identifier;
    this->pinNumber = pinNumber;
    this->type = type;
    this->parentIC = parentIC;
    this->parentIC_PortNumber = parentIC_PortNumber;
}

void ExtendedPin::pinset(int pinmode){
    ////Serial.println("PARENT: Calling pinset");
  
    if(this->type == "Inherited"){
      ////Serial.println("\t - Inherited pin");
      pinMode(pinNumber, pinmode);
    }
    else if(this->type == "DEMUX"){
      ////Serial.println("\t - DeMuxed pin");
      DMUX* parentDemux = (DMUX *)parentIC;
      parentDemux->setmodepin(parentIC_PortNumber,pinmode);
    }
    else if(this->type == "DIOE"){
      ////Serial.println("\t - Digital Expanded pin");
      IOE* parentIOE = (IOE *)parentIC;
      parentIOE->setmodegpio(parentIC_PortNumber,pinmode);
    }
    else if(this->type == "AIOE"){
      ////Serial.println("\t - Analog Expanded pin");
      int ooops;
    }
}

int ExtendedPin::pinread(){
    ////Serial.println("PARENT: Calling pinread");

    int readvalue;
  
    if(this->type == "Inherited"){
      ////Serial.println("\t - Inherited pin");
      readvalue = digitalRead(parentIC_PortNumber);
    }
    else if(this->type == "DEMUX"){
      ////Serial.println("\t - DeMuxed pin");
      DMUX* parentDemux = (DMUX *)parentIC;
      readvalue = parentDemux->readpin(parentIC_PortNumber);
    }
    else if(this->type == "DIOE"){
      ////Serial.println("\t - Digital Expanded pin");
      IOE* parentIOE = (IOE *)parentIC;
      readvalue = parentIOE->readgpio(parentIC_PortNumber);
    }
    else if(this->type == "AIOE"){
      ////Serial.println("\t - Analog Expanded pin");
      int ooops;
    }
    return readvalue;
}

void ExtendedPin::pinwrite(bool wvalue){
    ////Serial.println("PARENT: Calling pinwrite");
  
    if(this->type == "Inherited"){
      ////Serial.println("\t - Inherited pin");
      digitalWrite(parentIC_PortNumber, wvalue);
    }
    else if(this->type == "DEMUX"){
      ////Serial.println("\t - DeMuxed pin");
      DMUX* parentDemux = (DMUX *)parentIC;
      parentDemux->writepin(parentIC_PortNumber,wvalue);
    }
    else if(this->type == "DIOE"){
      ////Serial.println("\t - Digital Expanded pin");
      IOE* parentIOE = (IOE *)parentIC;
      parentIOE->writegpio(parentIC_PortNumber,wvalue);
    }
    else if(this->type == "AIOE"){
      ////Serial.println("\t - Analog Expanded pin");
      int ooops;
    }
}

void extended_pinMode(int pinNumber, int pinmode){
  ExtendedPin* pin = pinLocator(pinNumber);
  pin->pinset(pinmode);
}

void extended_digitalWrite(int pinNumber, bool wvalue){
  ExtendedPin* pin = pinLocator(pinNumber);
  pin->pinwrite(wvalue);
}

int extended_digitalRead(int pinNumber){
  ExtendedPin* pin = pinLocator(pinNumber);
  return pin->pinread();
}


// ---- Inherited Pins
//std::string identifier, int pinNumber, std::string type, void* parentIC, int parentIC_PortNumber
ExtendedPin GPIO1("GPIO1",1,INHE,nullptr,0);
ExtendedPin GPIO2("GPIO2",2,INHE,nullptr,0);
ExtendedPin GPIO4("GPIO4",4,INHE,nullptr,0);
ExtendedPin GPIO5("GPIO5",5,INHE,nullptr,0);
ExtendedPin GPIO6("GPIO6",6,INHE,nullptr,0);
ExtendedPin GPIO7("GPIO7",7,INHE,nullptr,0);
ExtendedPin GPIO8("GPIO8",8,INHE,nullptr,0);
ExtendedPin GPIO9("GPIO9",9,INHE,nullptr,0);
ExtendedPin GPIO10("GPIO10",10,INHE,nullptr,0);
ExtendedPin GPIO11("GPIO11",11,INHE,nullptr,0);
ExtendedPin GPIO12("GPIO12",12,INHE,nullptr,0);
ExtendedPin GPIO13("GPIO13",13,INHE,nullptr,0);
ExtendedPin GPIO14("GPIO14",14,INHE,nullptr,0);
ExtendedPin GPIO15("GPIO15",15,INHE,nullptr,0);
ExtendedPin GPIO16("GPIO16",16,INHE,nullptr,0);
ExtendedPin GPIO17("GPIO17",17,INHE,nullptr,0);
ExtendedPin GPIO18("GPIO18",18,INHE,nullptr,0);
ExtendedPin GPIO21("GPIO21",21,INHE,nullptr,0);
ExtendedPin GPIO36("GPIO36",36,INHE,nullptr,0);
ExtendedPin GPIO37("GPIO37",37,INHE,nullptr,0);
ExtendedPin GPIO38("GPIO38",38,INHE,nullptr,0);
ExtendedPin GPIO47("GPIO47",47,INHE,nullptr,0);
ExtendedPin GPIO48("GPIO48",48,INHE,nullptr,0);


// ---- Additional PCB 1
//std::string identifier,int pinInputSignal1,int pinChannelSelector3,int pinChannelSelector2,int pinChannelSelector1,int pinChannelSelector0,int pinEnable
IC_74HC4067 IC_DMADD1("DeMuxer Additional PCB 1",18,5,6,7,15,17);
//std::string identifier,std::bitset<7> I2C_Address,int pinSCL,int pinSDA,int pinINT,int pinRST
IC_PCA9555 IC_IOEADD1("IO Expander Additional PCB 1",0b0100000,9,8,1,2);

//std::string identifier, int pinNumber, std::string type, void* parentIC, int parentIC_PortNumber
ExtendedPin IO0("IO0",112,DIOE,&IC_IOEADD1,0);
ExtendedPin IO1("IO1",113,DIOE,&IC_IOEADD1,1);
ExtendedPin IO2("IO2",114,DIOE,&IC_IOEADD1,2);
ExtendedPin IO3("IO3",115,DIOE,&IC_IOEADD1,3);
ExtendedPin IO4("IO4",116,DIOE,&IC_IOEADD1,4);
ExtendedPin IO5("IO5",117,DIOE,&IC_IOEADD1,5);
ExtendedPin IO6("IO6",118,DIOE,&IC_IOEADD1,6);
ExtendedPin IO7("IO7",119,DIOE,&IC_IOEADD1,7);
ExtendedPin IO8("IO8",120,DIOE,&IC_IOEADD1,8);
ExtendedPin IO9("IO9",121,DIOE,&IC_IOEADD1,9);
ExtendedPin IO10("IO10",122,DIOE,&IC_IOEADD1,10);
ExtendedPin IO11("IO11",123,DIOE,&IC_IOEADD1,11);
ExtendedPin IO12("IO12",124,DIOE,&IC_IOEADD1,12);
ExtendedPin IO13("IO13",125,DIOE,&IC_IOEADD1,13);
ExtendedPin IO14("IO14",126,DIOE,&IC_IOEADD1,14);
ExtendedPin IO15("IO15",127,DIOE,&IC_IOEADD1,15);

ExtendedPin M0("M0",128,DEMUX,&IC_DMADD1,0);
ExtendedPin M1("M1",129,DEMUX,&IC_DMADD1,1);
ExtendedPin M2("M2",130,DEMUX,&IC_DMADD1,2);
ExtendedPin M3("M3",131,DEMUX,&IC_DMADD1,3);
ExtendedPin M4("M4",132,DEMUX,&IC_DMADD1,4);
ExtendedPin M5("M5",133,DEMUX,&IC_DMADD1,5);
ExtendedPin M6("M6",134,DEMUX,&IC_DMADD1,6);
ExtendedPin M7("M7",135,DEMUX,&IC_DMADD1,7);
ExtendedPin M8("M8",136,DEMUX,&IC_DMADD1,8);
ExtendedPin M9("M9",137,DEMUX,&IC_DMADD1,9);
ExtendedPin M10("M10",138,DEMUX,&IC_DMADD1,10);
ExtendedPin M11("M11",139,DEMUX,&IC_DMADD1,11);
ExtendedPin M12("M12",140,DEMUX,&IC_DMADD1,12);
ExtendedPin M13("M13",141,DEMUX,&IC_DMADD1,13);
ExtendedPin M14("M14",142,DEMUX,&IC_DMADD1,14);
ExtendedPin M15("M15",143,DEMUX,&IC_DMADD1,15);


// ---- PCB DSMSMC pinout
//std::string identifier,int pinInputSignal1,int pinChannelSelector2,int pinChannelSelector1,int pinChannelSelector0,int pinEnable
IC_74HC4051 IC_DM1_ENDS("DeMuxer END-S MUX1",6,48,47,21,36);
IC_74HC4051 IC_DM1_STARTS("DeMuxer END-S MUX1",7,48,47,21,36);
IC_74HC4051 IC_DM1_STEP("DeMuxer END-S MUX1",15,48,47,21,36);
IC_74HC4051 IC_DM1_DIR("DeMuxer END-S MUX1",16,48,47,21,36);
//std::string identifier,int pinInputSignal2,int pinInputSignal1,int pinChannelSelector2,int pinChannelSelector1,int pinEnable
IC_74HC4052 IC_DM2_GPIO("DeMuxer GPIO MUX2",10,14,18,17,36);
//std::string identifier,std::bitset<7> I2C_Address,int pinSCL,int pinSDA,int pinINT,int pinRST
IC_PCA9555 IC_IOE1("IO Expander 1",0b0100000,9,8,1,2);
IC_PCA9555 IC_IOE2("IO Expander 2",0b0100001,9,8,1,2);
IC_PCAL9538A IC_IOE3("IO Expander 2",0b1110000,9,8,1,2);

//std::string identifier, int pinNumber, std::string type, void* parentIC, int parentIC_PortNumber
ExtendedPin ENDS_M4("ENDS_M4",144,DEMUX,&IC_DM1_ENDS,4);
ExtendedPin ENDS_M3("ENDS_M3",145,DEMUX,&IC_DM1_ENDS,6);
ExtendedPin ENDS_M2("ENDS_M2",146,DEMUX,&IC_DM1_ENDS,7);
ExtendedPin ENDS_M1("ENDS_M1",147,DEMUX,&IC_DM1_ENDS,5);
ExtendedPin ENDS_M8("ENDS_M8",148,DEMUX,&IC_DM1_ENDS,2);
ExtendedPin ENDS_M7("ENDS_M7",149,DEMUX,&IC_DM1_ENDS,1);
ExtendedPin ENDS_M6("ENDS_M6",150,DEMUX,&IC_DM1_ENDS,0);
ExtendedPin ENDS_M5("ENDS_M5",151,DEMUX,&IC_DM1_ENDS,3);

ExtendedPin STARTS_M4("STARTS_M4",152,DEMUX,&IC_DM1_STARTS,4);
ExtendedPin STARTS_M3("STARTS_M3",153,DEMUX,&IC_DM1_STARTS,6);
ExtendedPin STARTS_M2("STARTS_M2",154,DEMUX,&IC_DM1_STARTS,7);
ExtendedPin STARTS_M1("STARTS_M1",155,DEMUX,&IC_DM1_STARTS,5);
ExtendedPin STARTS_M8("STARTS_M8",156,DEMUX,&IC_DM1_STARTS,2);
ExtendedPin STARTS_M7("STARTS_M7",157,DEMUX,&IC_DM1_STARTS,1);
ExtendedPin STARTS_M6("STARTS_M6",158,DEMUX,&IC_DM1_STARTS,0);
ExtendedPin STARTS_M5("STARTS_M5",159,DEMUX,&IC_DM1_STARTS,3);

ExtendedPin STEP_M4("STEP_M4",160,DEMUX,&IC_DM1_STEP,4);
ExtendedPin STEP_M3("STEP_M3",161,DEMUX,&IC_DM1_STEP,6);
ExtendedPin STEP_M2("STEP_M2",162,DEMUX,&IC_DM1_STEP,7);
ExtendedPin STEP_M1("STEP_M1",163,DEMUX,&IC_DM1_STEP,5);
ExtendedPin STEP_M8("STEP_M8",164,DEMUX,&IC_DM1_STEP,2);
ExtendedPin STEP_M7("STEP_M7",165,DEMUX,&IC_DM1_STEP,1);
ExtendedPin STEP_M6("STEP_M6",166,DEMUX,&IC_DM1_STEP,0);
ExtendedPin STEP_M5("STEP_M5",167,DEMUX,&IC_DM1_STEP,3);

ExtendedPin DIR_M4("DIR_M4",56,DEMUX,&IC_DM1_DIR,4);
ExtendedPin DIR_M3("DIR_M3",57,DEMUX,&IC_DM1_DIR,6);
ExtendedPin DIR_M2("DIR_M2",58,DEMUX,&IC_DM1_DIR,7);
ExtendedPin DIR_M1("DIR_M1",59,DEMUX,&IC_DM1_DIR,5);
ExtendedPin DIR_M8("DIR_M8",60,DEMUX,&IC_DM1_DIR,2);
ExtendedPin DIR_M7("DIR_M7",61,DEMUX,&IC_DM1_DIR,1);
ExtendedPin DIR_M6("DIR_M6",62,DEMUX,&IC_DM1_DIR,0);
ExtendedPin DIR_M5("DIR_M5",63,DEMUX,&IC_DM1_DIR,3);

ExtendedPin SLEEP_M1("SLEEP_M1",64,DIOE,&IC_IOE1,0);
ExtendedPin MS2_M1("MS2_M1",65,DIOE,&IC_IOE1,1);
ExtendedPin MS1_M1("MS1_M1",66,DIOE,&IC_IOE1,2);
ExtendedPin ENABLE_M1("ENABLE_M1",67,DIOE,&IC_IOE1,3);
ExtendedPin SLEEP_M2("SLEEP_M2",68,DIOE,&IC_IOE1,4);
ExtendedPin MS2_M2("MS2_M2",69,DIOE,&IC_IOE1,5);
ExtendedPin MS1_M2("MS1_M2",70,DIOE,&IC_IOE1,6);
ExtendedPin ENABLE_M2("ENABLE_M2",71,DIOE,&IC_IOE1,7);
ExtendedPin ENABLE_M8("ENABLE_M8",72,DIOE,&IC_IOE1,8);
ExtendedPin MS1_M8("MS1_M8",73,DIOE,&IC_IOE1,9);
ExtendedPin MS2_M8("MS2_M8",74,DIOE,&IC_IOE1,10);
ExtendedPin SLEEP_M8("SLEEP_M8",75,DIOE,&IC_IOE1,11);
ExtendedPin ENABLE_M7("ENABLE_M7",76,DIOE,&IC_IOE1,12);
ExtendedPin MS1_M7("MS1_M7",77,DIOE,&IC_IOE1,13);
ExtendedPin MS2_M7("MS2_M7",78,DIOE,&IC_IOE1,14);
ExtendedPin SLEEP_M7("SLEEP_M7",79,DIOE,&IC_IOE1,15);

ExtendedPin SLEEP_M3("SLEEP_M3",80,DIOE,&IC_IOE2,0);
ExtendedPin MS2_M3("MS2_M3",81,DIOE,&IC_IOE2,1);
ExtendedPin MS1_M3("MS1_M3",82,DIOE,&IC_IOE2,2);
ExtendedPin ENABLE_M3("ENABLE_M3",83,DIOE,&IC_IOE2,3);
ExtendedPin SLEEP_M4("SLEEP_M4",84,DIOE,&IC_IOE2,4);
ExtendedPin MS2_M4("MS2_M4",85,DIOE,&IC_IOE2,5);
ExtendedPin MS1_M4("MS1_M4",86,DIOE,&IC_IOE2,6);
ExtendedPin ENABLE_M4("ENABLE_M4",87,DIOE,&IC_IOE2,7);
ExtendedPin ENABLE_M6("ENABLE_M6",88,DIOE,&IC_IOE2,8);
ExtendedPin MS1_M6("MS1_M6",89,DIOE,&IC_IOE2,9);
ExtendedPin MS2_M6("MS2_M6",90,DIOE,&IC_IOE2,10);
ExtendedPin SLEEP_M6("SLEEP_M6",91,DIOE,&IC_IOE2,11);
ExtendedPin ENABLE_M5("ENABLE_M5",92,DIOE,&IC_IOE2,12);
ExtendedPin MS1_M5("MS1_M5",93,DIOE,&IC_IOE2,13);
ExtendedPin MS2_M5("MS2_M5",94,DIOE,&IC_IOE2,14);
ExtendedPin SLEEP_M5("SLEEP_M5",95,DIOE,&IC_IOE2,15);

ExtendedPin GPIOE1("GPIOE-1",96,DIOE,&IC_IOE3,0);
ExtendedPin GPIOE2("GPIOE-2",97,DIOE,&IC_IOE3,1);
ExtendedPin GPIOE3("GPIOE-3",98,DIOE,&IC_IOE3,2);
ExtendedPin GPIOE4("GPIOE-4",99,DIOE,&IC_IOE3,3);
ExtendedPin GPIOE5("GPIOE-5",100,DIOE,&IC_IOE3,4);
ExtendedPin GPIOE6("GPIOE-6",101,DIOE,&IC_IOE3,5);
ExtendedPin GPIOE7("GPIOE-7",102,DIOE,&IC_IOE3,6);
ExtendedPin GPIOE8("GPIOE-8",103,DIOE,&IC_IOE3,7);

ExtendedPin CS1("CS1",104,DEMUX,&IC_DM2_GPIO,1);
ExtendedPin CS3("CS3",105,DEMUX,&IC_DM2_GPIO,5);
ExtendedPin CS4("CS4",106,DEMUX,&IC_DM2_GPIO,7);
ExtendedPin CS2("CS2",107,DEMUX,&IC_DM2_GPIO,3);
ExtendedPin OUTPWM3("OUTPWM3",108,DEMUX,&IC_DM2_GPIO,4);
ExtendedPin OUTPWM2("OUTPWM2",109,DEMUX,&IC_DM2_GPIO,2);
ExtendedPin OUTPWM1("OUTPWM1",110,DEMUX,&IC_DM2_GPIO,0);
ExtendedPin OUTPWM4("OUTPWM4",111,DEMUX,&IC_DM2_GPIO,6);

ExtendedPin* pinLocator(int pinNumber){ //Temporary implementation. DELETE THIS IN FUTURE REVISIONS!
    switch(pinNumber){
      case 1: return &GPIO1;
      case 2: return &GPIO2;
      case 4: return &GPIO4;
      case 5: return &GPIO5;
      case 6: return &GPIO6;
      case 7: return &GPIO7;
      case 8: return &GPIO8;
      case 9: return &GPIO9;
      case 10: return &GPIO10;
      case 11: return &GPIO11;
      case 12: return &GPIO12;
      case 13: return &GPIO13;
      case 14: return &GPIO14;
      case 15: return &GPIO15;
      case 16: return &GPIO16;
      case 17: return &GPIO17;
      case 18: return &GPIO18;
      case 21: return &GPIO21;
      case 36: return &GPIO36;
      case 37: return &GPIO37;
      case 38: return &GPIO38;
      case 47: return &GPIO47;
      case 48: return &GPIO48;

      case 56: return &DIR_M4;
      case 57: return &DIR_M3;
      case 58: return &DIR_M2;
      case 59: return &DIR_M1;
      case 60: return &DIR_M8;
      case 61: return &DIR_M7;
      case 62: return &DIR_M6;
      case 63: return &DIR_M5;

      case 64: return &SLEEP_M1;// 
      case 65: return &MS2_M1;
      case 66: return &MS1_M1;
      case 67: return &ENABLE_M1;
      case 68: return &SLEEP_M2;//
      case 69: return &MS2_M2;
      case 70: return &MS1_M2;
      case 71: return &ENABLE_M2;
      case 72: return &ENABLE_M8;
      case 73: return &MS1_M8;
      case 74: return &MS2_M8;
      case 75: return &SLEEP_M8;//
      case 76: return &ENABLE_M7;
      case 77: return &MS1_M7;
      case 78: return &MS2_M7;
      case 79: return &SLEEP_M7;//

      case 80: return &SLEEP_M3;//
      case 81: return &MS2_M3;
      case 82: return &MS1_M3;
      case 83: return &ENABLE_M3;
      case 84: return &SLEEP_M4;//
      case 85: return &MS2_M4;
      case 86: return &MS1_M4;
      case 87: return &ENABLE_M4;
      case 88: return &ENABLE_M6;
      case 89: return &MS1_M6;
      case 90: return &MS2_M6;
      case 91: return &SLEEP_M6;//
      case 92: return &ENABLE_M5;
      case 93: return &MS1_M5;
      case 94: return &MS2_M5;
      case 95: return &SLEEP_M5;//

      case 96: return &GPIOE1;
      case 97: return &GPIOE2;
      case 98: return &GPIOE3;
      case 99: return &GPIOE4;
      case 100: return &GPIOE5;
      case 101: return &GPIOE6;
      case 102: return &GPIOE7;
      case 103: return &GPIOE8;

      case 104: return &CS1;
      case 105: return &CS3;
      case 106: return &CS4;
      case 107: return &CS2;
      case 108: return &OUTPWM3;
      case 109: return &OUTPWM2;
      case 110: return &OUTPWM1;
      case 111: return &OUTPWM4;

      case 112: return &IO0;
      case 113: return &IO1;
      case 114: return &IO2;
      case 115: return &IO3;
      case 116: return &IO4;
      case 117: return &IO5;
      case 118: return &IO6;
      case 119: return &IO7;
      case 120: return &IO8;
      case 121: return &IO9;
      case 122: return &IO10;
      case 123: return &IO11;
      case 124: return &IO12;
      case 125: return &IO13;
      case 126: return &IO14;
      case 127: return &IO15;

      case 128: return &M0;
      case 129: return &M1;
      case 130: return &M2;
      case 131: return &M3;
      case 132: return &M4;
      case 133: return &M5;
      case 134: return &M6;
      case 135: return &M7;
      case 136: return &M8;
      case 137: return &M9;
      case 138: return &M10;
      case 139: return &M11;
      case 140: return &M12;
      case 141: return &M13;
      case 142: return &M14;
      case 143: return &M15;

      case 144: return &ENDS_M4;
      case 145: return &ENDS_M3;
      case 146: return &ENDS_M2;
      case 147: return &ENDS_M1;
      case 148: return &ENDS_M8;
      case 149: return &ENDS_M7;
      case 150: return &ENDS_M6;
      case 151: return &ENDS_M5;

      case 152: return &STARTS_M4;
      case 153: return &STARTS_M3;
      case 154: return &STARTS_M2;
      case 155: return &STARTS_M1;
      case 156: return &STARTS_M8;
      case 157: return &STARTS_M7;
      case 158: return &STARTS_M6;
      case 159: return &STARTS_M5;

      case 160: return &STEP_M4;
      case 161: return &STEP_M3;
      case 162: return &STEP_M2;
      case 163: return &STEP_M1;
      case 164: return &STEP_M8;
      case 165: return &STEP_M7;
      case 166: return &STEP_M6;
      case 167: return &STEP_M5;
    }

}
