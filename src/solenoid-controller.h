#define SOLENOID_BREW_PIN 21
#define OMRON_CHANNEL_2 19


void setupSolenidController();
void turnBrewSwitchOn();
void turnBrewSwitchOff();
bool getSolenoidStatus();