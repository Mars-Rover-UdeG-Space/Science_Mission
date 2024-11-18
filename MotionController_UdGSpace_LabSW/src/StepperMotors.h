/** Stepmotors Test
 *  Version: 0.01
 *  Author: Jesus Delgadillo
 *  Description: This code contains structs, classes and methods for controlling A4988 drivers, startup of stepper motors, code debugging and a serial communication
 *  interface useful for test environments.
**/

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#include "PCB_controll.hpp"

#include <RosComands.hpp>

#include <std_msgs/Bool.h>

class StepMotor;

//Helper functions
std::string btos(bool x)
{
  if(x) return "True 1";
  return "False 0";
}

int btoint(bool x){
  if(x) return HIGH;
  return LOW;
}

//Library structs
struct LibA4988{
    //NOTE: HIGH and LOW are int values. HIGH = 0x1, LOW 0x0
    std::string string_modes[5] = {"fullstep","halfstep","quarterstep","eightstep","sixteenthstep"};
    int fullstep[3] = {LOW,LOW,LOW};
    int halfstep[3] = {HIGH,LOW,LOW};
    int quarterstep[3] = {LOW,HIGH,LOW};
    int eightstep[3] = {HIGH,HIGH,LOW};
    int sixteenthstep[3] = {HIGH,HIGH,HIGH};
    int speed_delay[18] = {20000,18000,16000,14000,12000,10000,8000,6000,4000,3500,3000,2500,2250,2200,2150,2100,2050,800};
} obj_LibA4988;

struct PinoutA4988{
    int pinEN, pinMS1, pinMS2, pinMS3, pinDIR, pinSTEP;
};

//Struct Declarations
struct mainParameters{
    int cerberus = 1; //Debug Level
    unsigned long Clock = 0; //Time Variable

    //3 buttons, 1: Motor Switch, 2: Step DOWN, 3:Step UP
    int button1;
    int button2;
    int button3;
    StepMotor *motorObject; //motorObject used in ButtonCaller function.
} global_parameters;

struct FunctionParameters{
    int quantity = 0;
    char parameter1[16] = {'\n'};
    char parameter2[16] = {'\n'};
    char parameter3[16] = {'\n'};
    char parameter4[16] = {'\n'};
    char parameter5[16] = {'\n'};
    char parameter6[16] = {'\n'};
    char parameter7[16] = {'\n'};
    char parameter8[16] = {'\n'};
};

//Class Declarations

class NodeElement{
    private:
        std::string functionName;
        int functionLevel;
        std::string parameters="";

    public:
        NodeElement *before, *after=nullptr;

        NodeElement(std::string,int); //Base Constructor
        NodeElement(std::string,int,std::string); //Constructor with string variable for storing parameters

        void set_parameters(std::string);

        std::string get_functionName();
        int get_functionLevel();
        std::string get_parameters();
};

class LevelList{
    private:
        NodeElement *initialItem, *lastItem;

    public:
        LevelList(); //Constructor

        void appendnew(std::string,std::string);
        void deletelast(std::string);
        void das_teller(std::string message, int ident=0, std::string type="common", int level=0);

        void givefeedback(NodeElement*,char);

} processList_obj;

class StepMotor{
    private:
        //Basic Information
        bool mset = false; //mset indicates that pins are set.
        bool ready = false; //ready indicates wheter the step motor is fully initialized.

        std::string name = "SMO";
        std::string model = "";
        std::string driver = "none";
        float baseResolution = 0; //This number is obtained through tests. Is the resolution in full step mode.
        int availableModes = 5;
        int operationalMode = -1;
        int operationalSpeed = 0;

        //Pin Information
        PinoutA4988 *driverPinoutA4988;
        int limMin, limMax; //Limits
        int UP, DOWN;

        float stepResolution;
        bool isCalibrated = false;
        long int stepCounter = 0;
        long int minPosition = 0; //Min position is the positional reference.
        long int maxPosition;
        long int currentPosition = -1;
        bool collisionStatus = true; //TRUE clear to go, FALSE about to collide.
        bool collisionDirection; //TRUE Max direction, FALSE, Min direction.
        int collisionCounter= 0;

    public:
        StepMotor();

        void stepper_enable();
        void stepper_disable();
        void stepper_initialize(std::string,std::string,float,int,std::string,int); //Acts as a constructor.
        void stepper_pinassign(std::string,int,int,int,int,int,int,int,int,bool);
        void stepper_recalibrate(bool);
        void stepper_rotate(int, bool);
        void stepper_goto(long int,bool);
        void stepper_gotomin();
        void stepper_gotomax();

        bool stepper_modeselector(std::string);

        void set_operationalSpeed(int);

        bool is_stepperatlimit();
        long int get_steppercurrentposition();

        bool get_mset(){return this->mset;};
        bool get_ready(){return this->ready;};

        std::string get_name(){return this->name;};
        std::string get_model(){return this->model;};
        std::string get_driver(){return this->driver;};
        float get_baseResolution(){return this->baseResolution;};
        int get_availableModes(){return this->availableModes;};
        int get_operationalMode(){return this->operationalMode;};
        int get_operationalSpeed(){return this->operationalSpeed;};

        //Pin Information
        PinoutA4988* get_driverPinoutA4988(){return this->driverPinoutA4988;};
        int get_limMin(){return this->limMin;};
        int get_limMax(){return this->limMax;}; //Limits

        float get_stepResolution(){return this->stepResolution;};
        bool get_isCalibrated(){return this->isCalibrated;};
        long int get_stepCounter(){return this->stepCounter;};
        long int get_minPosition(){return this->minPosition;}; //Min position is the positional reference.
        long int get_maxPosition(){return this->maxPosition;};
        long int get_currentPosition(){return this->currentPosition;};
        bool get_collisionStatus(){return this->collisionStatus;}; //TRUE clear to go, FALSE about to collide.
        bool get_collisionDirection(){return this->collisionDirection;}; //TRUE Max direction, FALSE, Min direction.
        int get_collisionCounter(){return this->collisionCounter;};
};

//Object Instances
StepMotor M1V,M2V,M1H;

/* ----------------------- LevelList Methods ----------------------------------------- */

LevelList::LevelList(){ //The list is initialized with one element.
    initialItem = new NodeElement("ParentFunction",1);
    initialItem->before = nullptr;
    lastItem = initialItem;
}

void LevelList::appendnew(std::string functionName, std::string parameters){
    NodeElement *newItem = new NodeElement(functionName,this->lastItem->get_functionLevel()+1,parameters);
    givefeedback(newItem,'I');

    if(lastItem != nullptr){
        lastItem->after = newItem;
        newItem->before = lastItem;
    }else{
        newItem->before = nullptr;
        initialItem = newItem;
    }

    lastItem = newItem;
}

void LevelList::deletelast(std::string parameters){
    if(lastItem != nullptr){

        lastItem->set_parameters(parameters);
        givefeedback(lastItem,'E');

        if(lastItem != initialItem){
            NodeElement *preItem = lastItem->before;
            preItem->after = nullptr;
            delete(lastItem);
            lastItem = preItem;
        } else{
            delete(lastItem);
            lastItem = nullptr;
            initialItem = nullptr;
        }
    }

}

void LevelList::givefeedback(NodeElement * Node, char mode){

    if(global_parameters.cerberus != 0){
        if(global_parameters.cerberus == 1 || global_parameters.cerberus == 2){
          if(mode == 'I'){
            if(Node->get_functionLevel() == 2){
                //Serialprint("---------------------\n");
                std::string output = "Call Initialization: " + std::to_string(Node->get_functionLevel()) + " - " + Node->get_functionName();
                das_teller(output);
                output = "Parameters: " + Node->get_parameters();
                das_teller(output);
            }
          }
          else if(mode == 'E'){
              if(Node->get_functionLevel() == 2){
                //Serialprint("-------------\n");
                std::string output = "Call End: " + std::to_string(Node->get_functionLevel()) + " - " + Node->get_functionName();
                das_teller(output);
                output = "Returned: " + Node->get_parameters();
                das_teller(output);
              }
          }
        }

        if(global_parameters.cerberus > 1){
           if(global_parameters.cerberus > 2){
              if(mode == 'I'){
                    //Serialprint("---------------------\n");
                    std::string output = "Call Initialization: " + std::to_string(Node->get_functionLevel()) + " - " + Node->get_functionName();
                    das_teller(output);
                    output = "Parameters: " + Node->get_parameters();
                    das_teller(output);
              }
              else if(mode == 'E'){
                    //Serialprint("-------------\n");
                    std::string output = "Call End: " + std::to_string(Node->get_functionLevel()) + " - " + Node->get_functionName();
                    das_teller(output);
                    output = "Returned: " + Node->get_parameters();
                    das_teller(output);
              }
           }
            std::string str_object = Node->get_functionName().substr(0,3);
            if(str_object.compare("SMO") == 0 || str_object.compare("M1V") == 0 || str_object.compare("M2V") == 0 || str_object.compare("M1H") == 0){
                //Its a method which belongs to a step motor.
                if(str_object.compare("M1V") == 0 || str_object.compare("SMO") == 0){
                  das_teller("operationalMode = " + std::to_string(M1V.get_operationalMode()));
                  das_teller("operationalSpeed = " + std::to_string(M1V.get_operationalSpeed()) + " = " + std::to_string(obj_LibA4988.speed_delay[M1V.get_operationalSpeed()]) + " ms");
                  das_teller("stepResolution = " + std::to_string(M1V.get_stepResolution()));
                  das_teller("minPosition = " + std::to_string(M1V.get_minPosition()));
                  das_teller("maxPosition = " + std::to_string(M1V.get_maxPosition()));
                  das_teller("currentPosition = " + std::to_string(M1V.get_currentPosition()));
                  das_teller("Collision Status = " + btos(M1V.get_collisionStatus()));
                  das_teller("Collision Direction = "+ btos(M1V.get_collisionDirection()));
                  das_teller("Collision Counter = " + std::to_string(M1V.get_collisionCounter()));
                }
                if(str_object.compare("M2V") == 0 || str_object.compare("SMO") == 0){
                  das_teller("operationalMode = " + std::to_string(M2V.get_operationalMode()));
                  das_teller("operationalSpeed = " + std::to_string(M2V.get_operationalSpeed()) + " = " + std::to_string(obj_LibA4988.speed_delay[M1V.get_operationalSpeed()]) + " ms");
                  das_teller("stepResolution = " + std::to_string(M2V.get_stepResolution()));
                  das_teller("minPosition = " + std::to_string(M2V.get_minPosition()));
                  das_teller("maxPosition = " + std::to_string(M2V.get_maxPosition()));
                  das_teller("currentPosition = " + std::to_string(M2V.get_currentPosition()));
                  das_teller("Collision Status = " + btos(M2V.get_collisionStatus()));
                  das_teller("Collision Direction = "+ btos(M2V.get_collisionDirection()));
                  das_teller("Collision Counter = " + std::to_string(M2V.get_collisionCounter()));

                }
                if(str_object.compare("M1H") == 0 || str_object.compare("SMO") == 0){
                  das_teller("operationalMode = " + std::to_string(M1H.get_operationalMode()));
                  das_teller("operationalSpeed = " + std::to_string(M1H.get_operationalSpeed()) + " = " + std::to_string(obj_LibA4988.speed_delay[M1V.get_operationalSpeed()]) + " ms");
                  das_teller("stepResolution = " + std::to_string(M1H.get_stepResolution()));
                  das_teller("minPosition = " + std::to_string(M1H.get_minPosition()));
                  das_teller("maxPosition = " + std::to_string(M1H.get_maxPosition()));
                  das_teller("currentPosition = " + std::to_string(M1H.get_currentPosition()));
                  das_teller("Collision Status = " + btos(M1H.get_collisionStatus()));
                  das_teller("Collision Direction = "+ btos(M1H.get_collisionDirection()));
                  das_teller("Collision Counter = " + std::to_string(M1H.get_collisionCounter()));
                }

            }
        }
    }

}

void LevelList::das_teller(std::string message, int ident, std::string type, int level) //Level = 0 means always display.
{
  if(level >= 0){
      if(type == "common"){
          if(ident == 1){
              //Serialprint("\t");
          }
      } else if(type == "verbose"){
            //Serialprint("MESSAGE: ");
      } else if(type == "warning"){
            //Serialprint("WARNING: ");
      } else if(type == "error"){
            //Serialprint("ERROR: ");
      } else if(type == "debug"){
            //Serialprint("DEBUG: ");
      }
        //Serialprintln(message.data());
  }
}

/* ----------------------- NodeElement Methods ----------------------------------------- */

NodeElement::NodeElement(std::string functionName, int functionLevel){
    this->functionName = functionName;
    this->functionLevel = functionLevel;
}

NodeElement::NodeElement(std::string functionName, int functionLevel, std::string parameters){
    this->functionName = functionName;
    this->functionLevel = functionLevel;
    this->parameters = parameters;
}

std::string NodeElement::get_functionName(){
    return this->functionName;
}

int NodeElement::get_functionLevel(){
    return this->functionLevel;
}

std::string NodeElement::get_parameters(){
    return this->parameters;
}

void NodeElement::set_parameters(std::string parameters){
    this->parameters = parameters;
}

/* ----------------------- StepMotor Methods ----------------------------------------- */

StepMotor::StepMotor(){
}

void StepMotor::stepper_enable(){
    digitalWrite(driverPinoutA4988->pinEN,LOW);
}

void StepMotor::stepper_disable(){
    digitalWrite(driverPinoutA4988->pinEN,HIGH);
}

void StepMotor::stepper_initialize(std::string name, std::string model, float baseResolution, int availableModes, std::string operationalResolution, int operationalSpeed = 4){
    std::string inputParameters = "name = " + name + "; model = " + model + "; baseResolution = " + std::to_string(baseResolution) + "; availableModes = " + std::to_string(availableModes) + "; operationalResolution = " + operationalResolution + "; operationalSpeed = " + std::to_string(operationalSpeed);
    processList_obj.appendnew(this->name + " | " + std::string(__func__),inputParameters);

    this->name = name;
    this->model = model;
    this->baseResolution = baseResolution;

    if(driver == "A4988"){

        this->availableModes = availableModes;
        set_operationalSpeed(operationalSpeed);

        if(stepper_modeselector(operationalResolution) == false){
            processList_obj.das_teller("No mode was selected.\n",1,"error");
        }
        else{ //If the operationalResolution is valid.

            this->stepResolution = this->baseResolution/pow(2,this->operationalMode);

            //Callibration routine.
            stepper_gotomin(); //Traverses until the min postion is found.
            this->stepCounter = 0; //Resets the step counter.
            stepper_gotomax(); //Traverses until the max postion is found.
            this->maxPosition = this->stepCounter; //Assigns the max position.
            stepper_gotomin(); //Traverses back until the min postion is found.
            this->currentPosition = 0; //Assigns the current position.
            this->stepCounter = 0; //Resets the step counter.
            this->isCalibrated = true; //The stepmotor is callibrated an ready to be used.

            this->ready = true;
        }
    }
    else{
        processList_obj.das_teller("Driver is unassigned.\n",1,"error");
        this->mset = false;
    }

    std::string returnValue = "void";
    processList_obj.deletelast(returnValue);
}
/*Assign piout 
    driver to use 
*/
void StepMotor::stepper_pinassign(std::string driver, int pinEN, int pinMS1, int pinMS2,int pinMS3,int pinDIR,int pinSTEP, int limMin, int limMax, bool up){
    std::string inputParameters = "driver = " + driver + "; pinEN = " + std::to_string(pinEN) + "; pinMS1 = " + std::to_string(pinMS1) + "; pinMS2 = " + std::to_string(pinMS2) + "; pinMS3 = " + std::to_string(pinMS3) + "; pinDIR = " + std::to_string(pinDIR) + "; pinSTEP = " + std::to_string(pinSTEP) + "; limMin = " + std::to_string(limMin) + "; limMax = " + std::to_string(limMax) + "; up = " + btos(up);
    processList_obj.appendnew(this->name + " | " + std::string(__func__),inputParameters);

    this->mset = false;
    this->ready = false;

    this->driver = driver;
    if(driver == "A4988"){
        //Pins Initialization
        driverPinoutA4988 = new PinoutA4988;

        driverPinoutA4988->pinEN = pinEN;
        driverPinoutA4988->pinMS1 = pinMS1;
        driverPinoutA4988->pinMS2 = pinMS2;
        driverPinoutA4988->pinMS3 = pinMS3;
        driverPinoutA4988->pinDIR = pinDIR;
        driverPinoutA4988->pinSTEP = pinSTEP;
        this->limMin = limMin;
        this->limMax = limMax;

        pinMode(pinEN,OUTPUT);
        pinMode(pinMS1,OUTPUT);
        pinMode(pinMS2,OUTPUT);
        pinMode(pinMS3,OUTPUT);
        pinMode(pinDIR,OUTPUT);
        pinMode(pinSTEP,OUTPUT);
        // This pines makes a interruption motion
        pinMode(limMin,INPUT); //Limit min
        pinMode(limMax,INPUT); //Limit max

        digitalWrite(driverPinoutA4988->pinEN,HIGH); //HIGH input disables the driver.
        digitalWrite(driverPinoutA4988->pinMS1,LOW);
        digitalWrite(driverPinoutA4988->pinMS2,LOW);
        digitalWrite(driverPinoutA4988->pinMS3,LOW);
        digitalWrite(driverPinoutA4988->pinDIR,LOW);
        digitalWrite(driverPinoutA4988->pinSTEP,LOW);

        this->UP = btoint(up);
        this->DOWN = btoint(!up);

        this->mset = true;
    }else{
        processList_obj.das_teller("Invalid driver!",1,"error");
    }

    std::string returnValue = "void";
    processList_obj.deletelast(returnValue);
}

void StepMotor::stepper_recalibrate(bool returnToRelPos){
    std::string inputParameters = "returnToRelPos = " + btos(returnToRelPos);
    processList_obj.appendnew(this->name + " | " + std::string(__func__),inputParameters);

    if(this->mset == false){
        processList_obj.das_teller("Motor is unset!",1,"warning");
    }
    else{
        if(this->ready == false){
            processList_obj.das_teller("Motor wasn't initialized!",1,"error");
        }
        else{
            long int initialRelPos = this->currentPosition;

            stepper_gotomin();                     // Traverses until the min postion is found.
            long int tmpStepCounter = this->stepCounter;    // Keeps the step counter.
            stepper_gotomax();                     // Traverses until the max postion is found.
            this->maxPosition = this->stepCounter - tmpStepCounter; // Assigns the max position.
            stepper_gotomin();                     // Traverses back until the min postion is found.
            this->currentPosition = 0;             // Assigns the current position.

            if(returnToRelPos == true){
                stepper_goto(initialRelPos,false);
            }
            this->stepCounter = tmpStepCounter;    // Resets the step counter.
            this->isCalibrated = true;             // The stepmotor is callibrated an ready to de used.
        }
    }

    std::string returnValue = "void";
    processList_obj.deletelast(returnValue);
}

void StepMotor::stepper_gotomin(){
    std::string inputParameters = "empty;";
    processList_obj.appendnew(this->name + " | " + std::string(__func__),inputParameters);

    if(this->mset == false){
        processList_obj.das_teller("Motor is unset!",1,"warning");
    }
    else{
        if(this->driver == "A4988"){
            digitalWrite(driverPinoutA4988->pinDIR,DOWN); //Min direction
            digitalWrite(driverPinoutA4988->pinEN,LOW); //Enables the driver.

            while(digitalRead(this->limMin) == LOW){
              stepper_rotate(1,0); //Rotate Until the min position or the limit is found.
            }

            digitalWrite(driverPinoutA4988->pinEN,HIGH); //Disables the driver.
        }

        if(currentPosition != minPosition){
            processList_obj.das_teller("The min position couldn't be reached, reached a limit close to min instead.",1,"verbose");
        }
    }

    std::string returnValue = "void";
    processList_obj.deletelast(returnValue);
}

void StepMotor::stepper_gotomax(){
    std::string inputParameters = "empty;";
    processList_obj.appendnew(this->name + " | " + std::string(__func__),inputParameters);

    if(this->mset == false){
        processList_obj.das_teller("Motor is unset!",1,"warning");
    }
    else{
        if(this->driver == "A4988"){
            digitalWrite(driverPinoutA4988->pinDIR,UP); //Max direction
            digitalWrite(driverPinoutA4988->pinEN,LOW); //Enables the driver.

            while(digitalRead(this->limMax) == LOW){
              stepper_rotate(1,1); //Rotate Until the max position or the limit is found.
            }

          digitalWrite(driverPinoutA4988->pinEN,HIGH); //Disables the driver.
        }

        if(currentPosition != maxPosition){
            processList_obj.das_teller("The max position couldn't be reached, reached a limit close to max instead.",1,"verbose");
        }
    }

    std::string returnValue = "void";
    processList_obj.deletelast(returnValue);
}

void StepMotor::stepper_rotate(int nSteps, bool direction){
    std::string inputParameters = "nSteps = " + std::to_string(nSteps) + "; direction = " + btos(direction);
    processList_obj.appendnew(this->name + " | " + std::string(__func__),inputParameters);

    contador.data = 0;
    char msg[18] = "Stepper rotate...";
    info.data = msg;
    if(this->mset == false){
        processList_obj.das_teller("Motor is unset!",1,"warning");
    }
    else{
        if(this->driver == "A4988"){

            if(direction == false){
                digitalWrite(driverPinoutA4988->pinDIR,DOWN); //Min direction
            }
            else if(direction == true){
                digitalWrite(driverPinoutA4988->pinDIR,UP); //Max direction
            }

            if(collisionStatus == false){
                if(direction == this->collisionDirection){
                    collisionCounter++; //Increases the count of prevented collisions.
                }
            }

            for( int x=0 ; x < nSteps ; x++){
                //SM_state llega del suscriptor para detener los motores
                node.spinOnce();//Listener 
                if(direction == false && digitalRead(this->limMin) == LOW){
                    this->currentPosition--; //Decrements the current position.
                }else if(direction == true && digitalRead(this->limMax) == LOW){
                    this->currentPosition++; //Increments the current position.
                }else{
                    this->collisionStatus = false;
                    this->collisionDirection = direction;
                    processList_obj.das_teller("A limit has been reached, the remaining step(s) can't be performed in this direction.",1,"verbose");
                    char msgTopic[25] = "Hardware_Interruption";
                    info.data = msgTopic; 
                    node.loginfo(info.data);
                    break;
                }
                // If the status of motor is disable then break the loop and return to the main loop.
                if(!SM_state){
                    char msgTopic[20] = "SW_Interruption";
                    info.data = msgTopic; 
                    node.loginfo(info.data);
                    break;
                }
                digitalWrite(driverPinoutA4988->pinSTEP,HIGH);
                delayMicroseconds(obj_LibA4988.speed_delay[this->operationalSpeed]);
                digitalWrite(driverPinoutA4988->pinSTEP,LOW);
                delayMicroseconds(obj_LibA4988.speed_delay[this->operationalSpeed]);
                contador.data = contador.data + 1;
                stepperCont.publish(&contador);
                //node.loginfo(info.data);
            }
        }else{
            processList_obj.das_teller("Driver is unassigned.\n",1,"error");
            this->mset = false;
        }
    }

    std::string returnValue = "void";
    processList_obj.deletelast(returnValue);
}

void StepMotor::stepper_goto(long int nSteps, bool referencePosition){
    std::string inputParameters = "nSteps = " + std::to_string(nSteps) + "; referencePosition = " + btos(referencePosition);
    processList_obj.appendnew(this->name + " | " + std::string(__func__),inputParameters);

    if(this->mset == false){
        processList_obj.das_teller("Motor is unset!",1,"warning");
    }
    else{
        if(this->ready == false){
            processList_obj.das_teller("Driver wasn't initialized!",1,"error");
        }
        else{

            long int relativeReferenceSteps;
            if(referencePosition == false){ //min as reference
                relativeReferenceSteps = nSteps;
            }else if(referencePosition == true){ //max as reference
                relativeReferenceSteps = this->maxPosition - nSteps;
            }

            if(this->currentPosition < relativeReferenceSteps){
                long int aSteps = relativeReferenceSteps - this->currentPosition;
                if(this->driver == "A4988"){
                  digitalWrite(driverPinoutA4988->pinEN,LOW); //Enables the driver.
                  stepper_rotate(aSteps, true); // Rotate to max position.
                  digitalWrite(driverPinoutA4988->pinEN,HIGH); //Disables the driver.
                }
            }
            else if (this->currentPosition > relativeReferenceSteps)
            {
                long int aSteps = this->currentPosition - relativeReferenceSteps;
                if(this->driver == "A4988"){
                  digitalWrite(driverPinoutA4988->pinEN,LOW); //Enables the driver.
                  stepper_rotate(aSteps, false); // Rotate to min position.
                  digitalWrite(driverPinoutA4988->pinEN,HIGH); //Disables the driver.
                }
            }
            else //If positions are equal
            {
                processList_obj.das_teller("Nothing to do.",1,"verbose");
            }
        }
    }

    std::string returnValue = "void";
    processList_obj.deletelast(returnValue);
}

bool StepMotor::stepper_modeselector(std::string operationalResolution){
    std::string inputParameters = "operationalResolution = " + operationalResolution;
    processList_obj.appendnew(this->name + " | " + std::string(__func__),inputParameters);

    bool returning = false;

    if(this->mset == false){
        processList_obj.das_teller("Motor is unset!",1,"warning");
    } else{
        if(driver == "A4988"){

            int previousOperationalMode = this->operationalMode;
            for(int i=0 ; i<=this->availableModes ; i++){ //Obtains the index of the selected microstep resolution.
                if(obj_LibA4988.string_modes[i] == operationalResolution){
                    this->operationalMode = i;
                }
            }
            if(this->operationalMode == previousOperationalMode){
                processList_obj.das_teller("No mode was selected or changed.\n",1,"verbose");
                if(this->operationalMode == -1){
                    processList_obj.das_teller("Invalid mode.\n",1,"error");   //Invalid mode.
                }
                returning = true; //Mode unchanged but previous value is valid.
            }else{
                if(operationalResolution == "fullstep"){
                    digitalWrite(driverPinoutA4988->pinMS1,obj_LibA4988.fullstep[0]);
                    digitalWrite(driverPinoutA4988->pinMS2,obj_LibA4988.fullstep[1]);
                    digitalWrite(driverPinoutA4988->pinMS3,obj_LibA4988.fullstep[2]);
                } else if(operationalResolution == "halfstep"){
                    digitalWrite(driverPinoutA4988->pinMS1,obj_LibA4988.halfstep[0]);
                    digitalWrite(driverPinoutA4988->pinMS2,obj_LibA4988.halfstep[1]);
                    digitalWrite(driverPinoutA4988->pinMS3,obj_LibA4988.halfstep[2]);
                } else if(operationalResolution == "quarterstep"){
                    digitalWrite(driverPinoutA4988->pinMS1,obj_LibA4988.quarterstep[0]);
                    digitalWrite(driverPinoutA4988->pinMS2,obj_LibA4988.quarterstep[1]);
                    digitalWrite(driverPinoutA4988->pinMS3,obj_LibA4988.quarterstep[2]);
                } else if(operationalResolution == "eightstep"){
                    digitalWrite(driverPinoutA4988->pinMS1,obj_LibA4988.eightstep[0]);
                    digitalWrite(driverPinoutA4988->pinMS2,obj_LibA4988.eightstep[1]);
                    digitalWrite(driverPinoutA4988->pinMS3,obj_LibA4988.eightstep[2]);
                } else if(operationalResolution == "sixteenthstep"){
                    digitalWrite(driverPinoutA4988->pinMS1,obj_LibA4988.sixteenthstep[0]);
                    digitalWrite(driverPinoutA4988->pinMS2,obj_LibA4988.sixteenthstep[1]);
                    digitalWrite(driverPinoutA4988->pinMS3,obj_LibA4988.sixteenthstep[2]);
                }
                returning = true;
            }
        }else{
            processList_obj.das_teller("Driver is unassigned.\n",1,"error");
            this->mset = false;
            returning = false;
        }

    std::string returnValue = "bool : " + btos(returning);
    processList_obj.deletelast(returnValue);

  }
  return returning;
}

void StepMotor::set_operationalSpeed(int operationalSpeed){
    std::string inputParameters = "operationalSpeed = " + std::to_string(operationalSpeed);
    processList_obj.appendnew(this->name + " | " + std::string(__func__),inputParameters);

    if(this->driver == "A4988"){
        if(operationalSpeed <= sizeof(obj_LibA4988.speed_delay) ){
            this->operationalSpeed = operationalSpeed;
        }
        else{
            processList_obj.das_teller("Invalid speed for driver A4988.\n",1,"warning");
        }
    }
    else{
        processList_obj.das_teller("Driver is unassigned.\n",1,"error");
        this->mset = false;
    }

    std::string returnValue = "void";
    processList_obj.deletelast(returnValue);
}

bool StepMotor::is_stepperatlimit(){
    if(collisionStatus == false){ //About to collide.
        return true;
    }
    return false;
}

long int StepMotor::get_steppercurrentposition(){

    return this->currentPosition;
}
