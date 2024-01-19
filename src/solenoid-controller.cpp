#include <solenoid-controller.h>
#include <Arduino.h>

void setupSolenidController(){
    pinMode(SOLENOID_BREW_PIN, OUTPUT);
}

void turnBrewSwitchOn(){
    if (getSolenoidStatus() == false){
        digitalWrite (SOLENOID_BREW_PIN, HIGH);
    }
}

void turnBrewSwitchOff(){
    if (getSolenoidStatus() == true){
        digitalWrite (SOLENOID_BREW_PIN, LOW);
    }
}

bool getSolenoidStatus(){
    return digitalRead(SOLENOID_BREW_PIN);
}
