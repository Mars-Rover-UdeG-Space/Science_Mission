
#include <Arduino.h>
#include "BTS7960.h"
#include "RosComands.hpp"
#include <iostream>

// Constantes del motor de la broca //
const uint8_t EN = 2;
const uint8_t L_PWM = 42;
const uint8_t R_PWM = 21;

BTS7960 motorController(EN, L_PWM, R_PWM);


unsigned long TimeWormMotor = 0;
int i;
struct L289N{
  const int IN3, IN4;
  bool bnd;
};

L289N control = {37, 1, true};
void mover(){
  digitalWrite(control.IN3, control.bnd);// 0 - horario  // 1 - antiorario
  digitalWrite(control.IN4, !control.bnd);// 1 
}

void stop(){
  digitalWrite(control.IN3, 0);
  digitalWrite(control.IN4, 0);
}

void invertir(){
  control.bnd = !control.bnd;
}

void direccion(bool BND){
  if(BND)
    control.bnd=true;
  else
    control.bnd=false;

}



void GusanoCallback(const std_msgs::UInt16& Gs){
  switch (Gs.data)
  {
  case 1:
  //Horario bajar
    motorController.Enable(); 
    control.bnd=true;
    mover();
   // delay(1000);
    TimeWormMotor = millis();
    break;
  case 2:
  //Subir
    control.bnd=false;
    mover();
    motorController.Enable();
    for(int speed = 0 ; speed < 255; speed+=10){
      motorController.TurnLeft(speed);
      delay(100);
    }

    //delay(1000);
  break;
  case 3:
    stop();
    motorController.Stop();
    std::cout<<TimeWormMotor<<std::endl;
    TimeWormMotor = 0;
    break;
  
  default:
    break;
  }

}

void collectorDownCallback(const std_msgs::UInt16& down){
  
  motorController.Enable();

  for(int speed = 0 ; speed < 255; speed+=10){
    motorController.TurnLeft(speed);
    delay(100);
  }  

  motorController.Stop();
  
  for(int speed = 255 ; speed > 0; speed-=10){
    motorController.TurnLeft(speed);
    delay(100);
  } 
   
  motorController.Stop();
  motorController.Disable();
  delay(1000); 
}