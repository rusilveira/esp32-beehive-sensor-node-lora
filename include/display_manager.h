#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include "dht_manager.h"

void initDisplay();
void updateDisplay(float pesoKg, const DHTReadings& dht);
void turnOffDisplay();
void turnOnDisplay();

#endif