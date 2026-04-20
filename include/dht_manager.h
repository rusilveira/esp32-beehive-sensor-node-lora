#ifndef DHT_MANAGER_H
#define DHT_MANAGER_H

#include <Arduino.h>

struct DHTReadings {
  float temperaturaInterna;
  float umidadeInterna;
  float temperaturaExterna;
  float umidadeExterna;

  bool internoValido;
  bool externoValido;
  bool leituraValida;

  unsigned long timestampMs;
};

void initDHT();
void updateDHT();

DHTReadings getDHTReadings();

float getTemperaturaInterna();
float getUmidadeInterna();
float getTemperaturaExterna();
float getUmidadeExterna();

bool isDHTInternoValido();
bool isDHTExternoValido();
bool isDHTLeituraValida();

#endif