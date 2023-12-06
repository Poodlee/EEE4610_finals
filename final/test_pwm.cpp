// pwm_test_with_onefile.cpp
#include <unistd.h>
#include <iostream>
#include <math.h>
#include <wiringPi.h>
#include <chrono>
#include <thread>

using namespace std;

class PWM {
 public:
  PWM(unsigned short pin);
  ~PWM();
  void setPin(unsigned short pin);
  void write(float duty);

 private:
  unsigned short pin;
};

PWM::PWM(unsigned short pin) {
  this->pin = pin;
  pinMode(pin,OUTPUT);
}
PWM::~PWM(){
  digitalWrite(pin, LOW);
}

void PWM::setPin(unsigned short pin) {
  this->pin = pin;
  pinMode(pin,OUTPUT);
}

void PWM::write(float duty){
    unsigned int delayHigh = round((20000 * duty));
    unsigned int delayLow = round(20000 * (1.0f - duty));
    digitalWrite(pin,HIGH);
    this_thread::sleep_for(chrono::microseconds(delayHigh));
    digitalWrite(pin, LOW);
    this_thread::sleep_for(chrono::microseconds(delayLow));
}


int main(){
  wiringPiSetupGpio();

  PWM THR_PWM(13),SER_PWM(18);

  for (int i = 0; i < 40; i++) {
    THR_PWM.write(0.072);
    SER_PWM.write(0.072);
  }
  while(1){
    // std::cout << " While in " << std::endl;
    std::this_thread::sleep_for(std::chrono::microseconds(20));
    SER_PWM.write(0.0726);
    THR_PWM.write(0.0777);
  }
  return 0;   
}