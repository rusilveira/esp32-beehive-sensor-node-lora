#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <Arduino.h>

struct PayloadColmeia {
  int16_t tempInterna_x100;
  uint16_t umidInterna_x100;
  int16_t tempExterna_x100;
  uint16_t umidExterna_x100;
  int32_t peso_x1000;
  uint8_t flags;
  uint8_t reservado;
};

void initRadio();
bool sendPacketReliable(const PayloadColmeia& payload);

#endif