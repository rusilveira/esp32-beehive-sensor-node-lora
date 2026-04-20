#ifndef LOADCELL_MANAGER_H
#define LOADCELL_MANAGER_H

#include <Arduino.h>

void initLoadCell();
void updateLoadCell();

float getPesoBrutoKg();
float getPesoFiltradoKg();
long getLoadCellOffset();
float getLoadCellFactor();

void tareLoadCell();
void startCalibrationMode();
void processLoadCellSerial();
void processLoadCellCommand(const String& entrada);

bool isLoadCellCalibrationWaiting();
bool isLoadCellPaused();

void powerDownLoadCell();
void powerUpLoadCell();

#endif