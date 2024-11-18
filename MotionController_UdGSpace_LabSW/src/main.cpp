/*Version 2: ROS integration for motion axis in measurements laboratory 
onboard the mars rover UdeG Space

Development team:
  - Oscar Alberto Gallo García
  - Andrés Alberto Ramirez Beltrán
  - Erick Emanuel Grimaldo Arteaga
  - Jesús Delgadillo Gaytan

This program use ROS topics for connection between microcontroller onboard
and the principal computer onboard (JETSON nano or Raspberry PI)

The functionality depends of the JETSON inputs in the serial node


*/
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <Wire.h>

#include "ClassMotors.hpp"

#include "RosComands.hpp"

#include "CollectorModule.hpp"

//NodoROS nodo_ros();

/*
  File A
  File B
  

*/

StepperMotors Z_stp("Z", 17, 16, 1000);


void setup(){

  //Z_stp.StepperConstructor("Z", 17, 16, 1000);
  
  //Set up movment worm motor
  pinMode(control.IN3, OUTPUT);
  pinMode(control.IN4, OUTPUT);

  // Ros setup
  node.initNode();

  //Subscribers
  node.subscribe(STOP);
  node.subscribe(StatusMachine);
  node.subscribe(calibration);
  node.subscribe(SubGus);
  node.subscribe(CollectorDown);

  node.subscribe(js); //Joy subscriber

  // Publishers 
  node.advertise(Slp);
  node.advertise(stepperCont);
  node.advertise(currentStatePico);

  //Sent this parameter to Principal Computer
  sleepMode.data = false;

// Test Topics

  node.subscribe(setDirWorm);
  
}

void loop(){
  node.spinOnce();
  delay(100);
}


void calibrationCallback(const std_msgs::UInt16& No){
  switch (No.data)
  {
  case 0:
    break;
  case 1:
    /*Calibrate state 1*/
    break;
  case 2:
    break;
  case 3:
    break;
  case 4:
    break;
  
  default:
  /*Calibrate all components*/
    break;
  }
}


void StatusMachineCallback(const std_msgs::UInt16& STATUS){
  if(!sleepMode.data){
    switch (STATUS.data)
    {
      case 1:
        /*
          State 1: Z-> DOWN
        */
        sleepMode.data = true;
        x1_msg.data = "Z-> DOWN...";
        Slp.publish(&sleepMode);//Send the sleep mode
        node.loginfo(x1_msg.data);
       // Z_stepper.stepper_rotate(500,true);
        Z_stp.Stp_rotate(6000, false);
        break;
      case 2:
        /*
          State 2: Collector Module
          1# Worm motor -> Down
          2# Excavator -> On
          3# Excavator -> Off
        */

        sleepMode.data = true;
        Slp.publish(&sleepMode);
        x1_msg.data = "Collector Module";
        node.loginfo(x1_msg.data);

        //Collector flow
        motorController.Enable();
        control.bnd = false;
        mover();
        motorController.TurnLeft(200);
        delay(5000);

        stop();//Worm Stop
        motorController.Stop(); // Collector stop
        motorController.Disable();
        break;
      case 3:
        //Worm Motor Up
        //Finish collection
        sleepMode.data = true;
        Slp.publish(&sleepMode);
        x1_msg.data = "Collector up";
        node.loginfo(x1_msg.data);
        control.bnd = true;
        mover();//Worm motor up
        delay(3000);
        stop();
        // Z -> axis up
   //     Z_stepper.stepper_rotate(6500,false);
                


        break;
      case 4:
      //Vuelta
        sleepMode.data = true;
        Slp.publish(&sleepMode);
        x2_msg.data = "X2 Right";
        node.loginfo(x2_msg.data);
    //    X2_stepper.stepper_rotate(65000,false);
        break;

      case 5:
        sleepMode.data = true;
        Slp.publish(&sleepMode);
        x2_msg.data = "X2 Left";
        node.loginfo(x2_msg.data);
    //    X2_stepper.stepper_rotate(6500,true);
        break;
      case 6:
        //Subir
        sleepMode.data = true;
        Slp.publish(&sleepMode);
        x2_msg.data = "Collector UP";
        node.loginfo(x2_msg.data);
        //STM3.stepper_rotate(6500,false);
        //delay(1000);
        break;
      case 7:
    //STM5 -> Riel bandejas
    //Adelante
        //STM5.stepper_rotate(6500,true);
        //delay(1000);
        break;
      case 8:
      //Bandejas -> atras
        //STM5.stepper_rotate(6500,false);
        //delay(1000);
        //break;

      case 9:
        //Motor Periquito
        break;
      case 10:
        //Motor Periquito
        break;
      case 11:
    //Motor Periquito Stop
        break;
      case 12:
        break;
      default:
        int stp = HIGH;
        //Forzar a detener mandando un estado en alto al pin de interrupcion.
     //   stp = digitalRead(16);
        break;
    }
    sleepMode.data = false;
    Slp.publish(&sleepMode);
  }

}


void JoyData(const sensor_msgs::Joy& joy){
  
  /*
    Xbox buttons 
  */
  bool A_btn = joy.buttons[0];
  bool B_btn = joy.buttons[1];
  bool X_btn = joy.buttons[2];
  bool Y_btn = joy.buttons[3];
  bool LB_btn = joy.buttons[4];
  bool RB_btn = joy.buttons[5];


  //Xbox axes



  if((joy.buttons[1]) == true){
    node.loginfo("Collector Enable...");
    motorController.Enable();
   // Z_stepper.stepper_rotate(6500,true);
    //Worm Mode 
    if(joy.axes[1] == 1){
      //Worm motor down
      node.loginfo("Worm up");
      control.bnd=true;
      mover();
    }else if(joy.axes[1] == -1){
      //Worm motor up
      node.loginfo("Worm down");
      control.bnd=false;
      mover();
    }else{
      //node.loginfo("stop worm motor");
      stop();
      motorController.Stop();
    }

    if(joy.buttons[7] == true && !joy.buttons[5]){
      node.loginfo("Drill Turn Left medium PWM");
      motorController.TurnLeft(128);
    }else if(joy.buttons[7] == false && joy.buttons[5] == true){
      node.loginfo("Drill Turn Left Full PWM");
      motorController.TurnLeft(255);
    }

    if (joy.buttons[6] && !joy.buttons[4]){
      node.loginfo("Drill Turn Right medium PWM");
      motorController.TurnRight(128);
    }else if(!joy.buttons[6] && joy.buttons[4]){
      node.loginfo("Drill Turn Right Full PWM");
      motorController.TurnRight(255);
    }


    if(!joy.buttons[6] && !joy.buttons[4] && !joy.buttons[7] && !joy.buttons[5]){
      motorController.Stop();
      motorController.Disable();
    }
  }else{
    //node.loginfo("Joy Disable");
    stop();
    motorController.Stop();
    motorController.Disable();
    //
  }

  if(joy.buttons[0]){
    node.loginfo("Stepper Enable...");
    SM_state = true;
    if(joy.axes[5] == -1){
      // Z axis down 
      node.loginfo("Z Stepper Down");
      //Z_stepper.stepper_rotate(6500, false);
     // Z_stp.Stp_JoyRotate(false);
      Z_stp.Stp_rotate(6500,false);
    }else if(joy.axes[5] == 1){
      node.loginfo("Z Stepper Up");
      //Z_stepper.stepper_rotate(6500, true);
      //Z_stp.Stp_JoyRotate(true);
      //Z_stp.Stp_rotate(6500,false);
    }

    if(joy.axes[4] == -1){
      node.loginfo("X2 Stepper Right");
      //X2_stepper.stepper_rotate(6500, true);
    }else if(joy.axes[4] == 1){
      node.loginfo("X2 Stepper Left");
      //X2_stepper.stepper_rotate(6500, false);
    }

    if(joy.axes[1] ==  -1){
      node.loginfo("Z2 Stepper Down");
      //Z2_stepper.stepper_rotate(6500,true);
    }else if(joy.axes[1] == 1){
      node.loginfo("Z2 Stepper Up");
      //Z2_stepper.stepper_rotate(6500,false);
    }
  }else{
    

  }


  if(joy.buttons[3]){
    node.loginfo("Stepper stop...");
    SM_state = false;
  }

}





