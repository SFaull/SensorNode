#include "FastLED.h"
#define LED_UPDATE_TIMEOUT     20   // update led every 20ms
#define PULSE_TIMEOUT 500

#define STEPS 25

class LEDController
{
  public:
    LEDController(CRGB *_leds);
    void run(void);
    void setColour(int r, int g, int b);
    void setColourTarget(int r, int g, int b);
    void pulse(int r, int g, int b);
    
  private:
    void setColourTransition(void);
    void fadeToColourTarget(void);
    void applyColour(uint8_t r, uint8_t g, uint8_t b);
    void setTimer(unsigned long *startTime);
    bool timerExpired(unsigned long startTime, unsigned long expiryTime);
    
    CRGB *leds;
    int ledCount;
    bool target_met;
    unsigned int target_colour[3];
    unsigned int current_colour[3];
    unsigned int transition[STEPS][3];

    unsigned long runTime;
    unsigned long ledTimer;
    unsigned long pulseTimer;
    bool isPulse;
};
