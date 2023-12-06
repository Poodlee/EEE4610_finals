// pwm.cpp
#include "desert_ant_navigation/pwm.hpp"
#include <unistd.h>
#include <iostream>
#include <fstream>
using namespace std;  

PWM::PWM(){}

void PWM::setpin(unsigned short pin){
  this->pin = pin;
  pinMode(pin,OUTPUT);
}
PWM::~PWM(){
  digitalWrite(pin, LOW);
}

void PWM::testing(float duty){
    unsigned int delayHigh = round((20000 * duty));
    unsigned int delayLow = round(20000 * (1.0f - duty));
    digitalWrite(pin,HIGH);
    this_thread::sleep_for(chrono::microseconds(delayHigh));
    digitalWrite(pin, LOW);
    this_thread::sleep_for(chrono::microseconds(delayLow));
}