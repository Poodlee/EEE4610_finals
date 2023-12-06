#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <wiringPi.h> 

int main() {
  wiringPiSetupGpio();

  int prev_num=0;
  while(1){
    // 파일에서 Pin Num과 Duty 값을 읽어옴
    
    pwmWrite(prev_num, 0);
    pwmWrite(prev_num,INPUT);
    // std::this_thread::sleep_for(std::chrono::milliseconds(150));


    // 주기만큼 delay를 해야 하지 않을까? 그래야 pwm 파형 주기가 맞아 떨어질 듯 
    std::ifstream inputFile("/home/ubuntu/desert_ant_navigation_ws/src/desert_ant_navigation/src/values.txt");
    if (inputFile.is_open()) {
      int pinNum;
      float Duty;
      inputFile >> pinNum;
      inputFile >> Duty;
      inputFile.close();

      int pwmRange = 1024;  
      int pwmClock = 375; 

      prev_num = pinNum;
      pinMode(pinNum, PWM_OUTPUT);
      pwmSetMode(PWM_MODE_MS);
      pwmSetClock(pwmClock);
 

      int pwmValue = static_cast<int>(Duty * pwmRange);
      pwmWrite(pinNum, pwmValue);

      // 읽어온 값들을 출력
      std::cout << "Read Pin Num: " << pinNum << "\n";
      std::cout << "Read Duty Value: " << Duty << "\n";




    //   //파일을 지움
    //   if (std::remove("values.txt") != 0) {
    //       std::cerr << "Error deleting the file.\n";
    //   } else {
    //       std::cout << "File deleted successfully.\n";
      // }
    } else {
        std::cerr << "Unable to open the file.\n";
    }
  }

    return 0;
}
