/*

                                | UdeG Space |
    | Mars Rover Laboratory | Software Development Team | Science Mission |

    Class for implementation Stepper Motors controll and use Arduino.h 
    for hardware implementation functions.

    Hardware List:
    - ESP32 S3 Devkit 1
    - Stepper Motor Nema17 
    - Driver ------
    - Xbox Type Control (Optional)

    /-----------------------------Instructions-----------------------------/
    Version 2.1
    In this version the implementation depends only for two digital pins of 
    the microcontroller. 

    /____________Constructor____________/

    pin step -> define the number of steps.
    pin dir  -> define the direction of the motion. 

    In the constructor only needs select 4 parameters
    1 - Object Name 
    2 - Pin Steps
    3 - Pin Direction
    4 - Delay, this is a constant integer who define the velocity of motor.
        This parameter have hardware dependency.

    /__________Rotate method___________/

    This method have 2 parameters:

    1 - Number of steps: Integer
    2 - Direction: Boolean 
    
    Note: If you want to use the Xbox control, you have to use other
    Rotate method optimizated for this specific work.

    Note 2: Use the same rotate method have hardware-level troubles.


*/

#include <Arduino.h>
#include <iostream>
#include "RosComands.hpp"
#include <std_msgs/Int32.h>

class StepperMotors{
    private:
        String Name; //The motor's name.
        bool start = false; //True if the motor is in the initial position, False in other cases.
        bool end = false; //True if the motor is in final of career, False in other cases.
        bool available = false; //True if the motor is stop, false if the motor is moving.

        int delay; //Velocity
        int pinStep; //Pin for steps.
        int pinDir; //Pin for Direction.
        
        struct Limit
        {
            int pinBtn;   
        };
        
        
        std_msgs::Int32 stepCounter;
        std_msgs::String stepperMessages;

    public:
        StepperMotors(String Name, int pinStep, int pinDir, int delay) 
            : Name(Name), pinStep(pinStep), pinDir(pinDir), delay(delay){
            this->Name = Name;
            this->delay = delay;
            this->pinDir = pinDir;
            this->pinStep = pinStep;
            this->stepCounter.data = 0;

            pinMode(pinDir, OUTPUT);
            pinMode(pinStep, OUTPUT);

            this->available = true;

            node.loginfo("is enable to use");
        }
       // void StepperConstructor(std::string Name, int pinStep, int pinDir, int delay);
        
        // Set and Get Methods:
        void setName(String Name){
            this->Name = Name;
        }
        String getName(){
            return this->Name;
        }
        void setStart(bool start){
            this->start = start;
        }
        bool getStart(){
            return this->start;
        }
        void setDelay(int delay){
            this->delay = delay;
        }
        int getDelay(){
            return this->delay;
        }
        void setPinStep(int pinStep){
            this->pinStep = pinStep;
        }
        int getPinStep(){
            return this->pinStep;
        }
        void setPinDir(int pinDir){
            this->pinDir = pinDir;
        }
        int getPinDir(){
            return this->pinDir;
        }
        void setStepCounter(std_msgs::Int32 stepCounter){
            this->stepCounter.data = stepCounter.data;
        }
        std_msgs::Int32 getStepCounter(){
            return this->stepCounter;
        }

        void Stp_rotate(int nSteps, bool direction);

        void Stp_JoyRotate(bool direction);
        
        
};

//StepperMotors::StepperMotors(){}

/*
    Constructor with parameters 
    Name -> Name for identify the stepper motor.
    pinStep -> Pin assingnation for controll steps.
    pinDir -> Pin assignation for controll direction.
    delay -> parameter for controll velocity.
*/
/* void StepperMotors::StepperConstructor(std::string Name, int pinStep, int pinDir, int delay){
    this->Name = Name;
    this->delay = delay;
    this->pinDir = pinDir;
    this->pinStep = pinStep;
    this->stepCounter.data = 0;

    pinMode(pinDir, OUTPUT);
    pinMode(pinStep, OUTPUT);

    this->available = true;

    node.loginfo("is enable to use");
} */


/*
    This is the main method in this clase, is with the motors move.

    Parameters:
    nSteps: the number of steps that move.
    direction: True go forward and false go back.
*/
void StepperMotors::Stp_rotate(int nSteps, bool direction){
    if(this->available){
        this->available = false;
        digitalWrite(this->pinDir, direction);
        for(int i = 0; i < nSteps; i++){
            node.spinOnce(); //Listen topics 
            if(!SM_state){ 
                this->available = true;
                //node.loginfo(this->Name);
                node.loginfo("SW_Interruption");   
                this->stepCounter.data = 0;
                break;
            }
            digitalWrite(this->pinStep, HIGH);
            delayMicroseconds(this->delay);
            digitalWrite(this->pinStep, LOW);
            delayMicroseconds(this->delay);
            this->stepCounter.data = i;
            stepperCont.publish(&stepCounter);
            node.loginfo("Test");   
        }
        this->available = true;
    }else{
        node.logwarn("Stepper is not available....");
    }
}

void StepperMotors::Stp_JoyRotate(bool direction){
    node.loginfo("Joy Rotate");
    digitalWrite(this->pinDir, direction);// Direction 
    //PWM
    digitalWrite(this->pinStep, HIGH);
    delayMicroseconds(this->delay);
    digitalWrite(this->pinStep, LOW);
    delayMicroseconds(this->delay);
    //Counter data
    contador.data = this->stepCounter.data++;
    //Working with this part of the code.
    stepperCont.publish(&contador);
}

//This class is to improve the structure for wormMotor motion
class WormMotor{
    private:
        const int pinIN3;
        const int pinIN4;
        std_msgs::Bool Dir;

    public:
    //How create a better constructor.
        WormMotor() : pinIN3(pinIN3), pinIN4(pinIN4){
            pinMode(pinIN3, OUTPUT);
            pinMode(pinIN4, OUTPUT);
            this->Dir.data = false;// If the module is correct connected this parameter is not necesary to adjust.
            // In other case we can edit this using a ROS Topic 
        };

      //We need to create a method for detect if the worm motor works
    
        void setDir(bool Dir){
            this->Dir.data = Dir;
        }

/*         void setDirWormCallback(const std_msgs::Bool& dir){

            //This I want to create a topic to change the direction of worm motor
            //This topic helps to reduce the error impact in the module connection, and solve with code.
        }

 */
        std_msgs::Bool getDir(){
            return this->Dir;
        }

        void Down(){
            digitalWrite(this->pinIN3, this->Dir.data);
            digitalWrite(this->pinIN4, !this->Dir.data);
        }

        void Up(){
            this->Dir.data = !this->Dir.data;
            this->Down();
        }

};

class ExcavatorMotor{
    private:
        const uint8_t Enable;
        const uint8_t L_pwm;
        const uint8_t R_pwm;

    public:
    



};
