// PWM.cpp
// Include the header file
#include "PWM.hpp"
#include <iostream>
#include <wiringPi.h>

// Constructor
PWMGenerator::PWMGenerator(int pin){
		pwmPin = pin;
}
// Destructor
PWMGenerator::~PWMGenerator() {
    stop();
}

// Initialize PWM
void PWMGenerator::initialize() {
    if (wiringPiSetupGpio() == -1) {
        std::cerr << "WiringPi initialization failed." << std::endl;
        return;
    }

    // Set the PWM pin to output mode
    pinMode(PWM_PIN, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);
    pwmSetClock(PWM_CLOCK);
    pwmSetRange(PWM_RANGE);
}

// Set PWM value based on the input value
// 2000(1.1ms) ~ 3000 (1.51) ~ 4000(1.9ms) 
void PWMGenerator::setPWMValue(int value) {
    if (value <= INPUT_MIN) {
        value = INPUT_MIN;
    } else if (value >= INPUT_MAX) {
        value = INPUT_MAX;
    }

		value = 
		// dutyCycle:1~1024 -> 원하는 FREQ의 duty cycle 결정
		// 2000(200) ~ 3000(280) ~ 4000(360)
		// 36.2 ~ 50.68 ~ 65.16
		value = 36.2 + (value-2000) * (65.16-36.2)/(4000-2000);
    int dutyCycle = 1000/PWM_FREQ * value;  
    pwmWrite(pwmPin, dutyCycle);
}

// Stop PWM by setting duty cycle to 0
void PWMGenerator::stop() {
    pwmWrite(pwmPin, 0);
}
