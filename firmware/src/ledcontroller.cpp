#include "ledcontroller.h"
#include "FastLED.h"

LEDController::LEDController(CRGB *_leds)
{
  isPulse = false;
  runTime = 0;
  ledTimer = 0;
  pulseTimer = 0;
  leds = _leds;
  ledCount =  sizeof(leds) / sizeof(CRGB);
  target_met = false;
  setColourTarget(0,0,0); // off
}

void LEDController::run(void)
{
    /* Periodically update the LEDs */
    if(timerExpired(ledTimer, LED_UPDATE_TIMEOUT))
    {
      setTimer(&ledTimer); // reset timer
      fadeToColourTarget();
    }

    if(timerExpired(pulseTimer, PULSE_TIMEOUT) && isPulse)
    {
      setColourTarget(0,0,0);
      isPulse = false;
    }
}


void LEDController::setColourTarget(int r, int g, int b)
{
  target_met = false;
  
  target_colour[0] = r;
  target_colour[1] = g;
  target_colour[2] = b;
  setColourTransition();
}

void LEDController::pulse(int r, int g, int b)
{
  target_met = false;
  isPulse = true;
  setTimer(&pulseTimer);
  target_colour[0] = r;
  target_colour[1] = g;
  target_colour[2] = b;
  setColourTransition();
}


void LEDController::setColourTransition(void)
{
  for(int addr=0; addr<STEPS; addr++)  // for each element in the array
  {

    for (int i=0; i<3; i++)  // for each colour in turn
    {
      if(addr == (STEPS - 1))
            transition[addr][i] = target_colour[i]; // ensure the last step is the exact target
      else
        transition[addr][i] = map(addr, 0, STEPS-1, current_colour[i], target_colour[i]); // compute the proportional colour value
    }

    #if 0
    Serial.print(transition[addr][0]);
    Serial.print(",");
    Serial.print(transition[addr][1]);
    Serial.print(",");
    Serial.println(transition[addr][2]);
    #endif
  }
}

void LEDController::fadeToColourTarget(void)
{
  static int addr = 0;
    
  if(!target_met)
  {
    setColour(transition[addr][0],transition[addr][1],transition[addr][2]);
    addr++;
    
    if (addr>=STEPS)
    {
      target_met = true;
      addr = 0;
    }      
  }
}

void LEDController::setColour(int r, int g, int b)
{
  current_colour[0] = r;
  current_colour[1] = g;
  current_colour[2] = b;
  applyColour(current_colour[0],current_colour[1],current_colour[2]);
}

void LEDController::applyColour(uint8_t r, uint8_t g, uint8_t b)
{
  if (r < 256 && g < 256 && b < 256)
  {
    for (uint8_t i=0; i<ledCount; i++)
    {
      leds[i] = CRGB(r,g,b); 
    }

    FastLED.show();
#if 0
    Serial.print("Whole strip set to ");
    Serial.print(r);
    Serial.print(",");
    Serial.print(g);
    Serial.print(",");
    Serial.println(b);
#endif
  }
  else
    Serial.println("Invalid RGB value, colour not set");
}

/* pass this function a pointer to an unsigned long to store the start time for the timer */
void LEDController::setTimer(unsigned long *startTime)
{
  runTime = millis();    // get time running in ms
  *startTime = runTime;  // store the current time
}

/* call this function and pass it the variable which stores the timer start time and the desired expiry time 
   returns true fi timer has expired */
bool LEDController::timerExpired(unsigned long startTime, unsigned long expiryTime)
{
  runTime = millis(); // get time running in ms
  if ( (runTime - startTime) >= expiryTime )
    return true;
  else
    return false;
}
