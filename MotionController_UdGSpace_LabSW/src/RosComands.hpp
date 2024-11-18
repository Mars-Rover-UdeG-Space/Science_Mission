#ifndef RosComands_hpp
#define RosComands_hpp

#include <ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Empty.h>
#include <sensor_msgs/Joy.h>

//ros::NodeHandle nodeStepper; 


ros::NodeHandle node;

//Mensajes Globales
std_msgs::String x1_msg;
std_msgs::String x2_msg;
std_msgs::String info;

std_msgs::Bool interruptor;

/*Callback functions*/
void StatusMachineCallback(const std_msgs::UInt16& STATUS);

void StopSMCallback(const std_msgs::Bool& Motor_Status);
//void writeMemoryCallback(const std_msgs::Empty& Value);
void calibrationCallback(const std_msgs::UInt16& No);
void GusanoCallback(const std_msgs::UInt16& Gs);
void collectorDownCallback(const std_msgs::UInt16& down);

void JoyData(const sensor_msgs::Joy& joy);

void setDirWormCallback(const std_msgs::Bool& Dir){
    bool x = Dir.data;
}


//Publishers
std_msgs::Bool sleepMode;
ros::Publisher Slp("Slp", &sleepMode);
std_msgs::UInt16 contador;
ros::Publisher stepperCont("stepperCont", &contador);

std_msgs::UInt16 currentState;
ros::Publisher currentStatePico("currentStatePico", &currentState);


//Subscribers
ros::Subscriber<std_msgs::UInt16>StatusMachine("StatusMachine", &StatusMachineCallback);
ros::Subscriber<std_msgs::Bool>STOP("StopStepperMotors", &StopSMCallback);
//ros::Subscriber<std_msgs::Empty>writeMemory("writeMemory", &writeMemoryCallback);

/*
    This topic can go to the initial position of any state in the machine
*/
ros::Subscriber<std_msgs::UInt16>calibration("calibration", &calibrationCallback);
ros::Subscriber<std_msgs::UInt16>SubGus("Gusano", &GusanoCallback);
ros::Subscriber<std_msgs::UInt16> CollectorDown("collectorDown", &collectorDownCallback);

//Topic to change the Dir of the Worm motor
ros::Subscriber<std_msgs::Bool>setDirWorm("setDir_Worm", &setDirWormCallback);

//ros::Subscriber<sensor_msgs::Joy> js("joy_roboclaw", JoyData);
ros::Subscriber<sensor_msgs::Joy> js("joy_roboclaw", JoyData);


bool SM_state = true;


void StopSMCallback(const std_msgs::Bool& Motor_Status){
    SM_state = Motor_Status.data;
   // digitalWrite(LED_BUILTIN, SM_state);
}

/*  
    This clase is use for tell if the ros node is enable.
*/
class NodoROS{
    private:
        std_msgs::Bool status;
        ros::Publisher statusMode;
        //ros::NodeHandle& node;
    public:
        NodoROS();
        
        void setStatus(std_msgs::Bool status){
            this->status = status;
        }
        std_msgs::Bool getStatus(){
            return status;
        }
        void publishStatus(){
            statusMode.publish(&status);
        }
};


#endif