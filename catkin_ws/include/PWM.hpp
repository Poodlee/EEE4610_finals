// PWM.hpp
#ifdef PWM_HPP
#define PWM_HPP

// Define the PWM frequency in Hz
#define PWM_FREQ 181

// Define the PWM clock divisor, PWM range 180 = 19.2M / 106 / 1000 = 181Hz (대략 180)
#define PWM_CLOCK 106
#define PWM_RANGE 1000

// Define the minimum and maximum input values
#define INPUT_MIN 2000
#define INPUT_MAX 4000

class PWMGenerator {
public:
    PWMGenerator(int pin);
    ~PWMGenerator();

    void initialize();
    void setPWMValue(int value);
    void stop();

private:
    int pwmPin;
};

#endif