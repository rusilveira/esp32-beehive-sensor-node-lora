#include "dht_manager.h"
#include "config.h"
#include <DHT.h>
#include <math.h>

static DHT dhtInterno(DHT_PIN_INTERNO, DHT_TYPE);
static DHT dhtExterno(DHT_PIN_EXTERNO, DHT_TYPE);

static DHTReadings leituras = {
  NAN, NAN, NAN, NAN,
  false, false, false,
  0
};

static unsigned long ultimoUpdate = 0;
static const unsigned long intervaloLeituraMs = 2000;

// -------------------------------------------------------------------
static bool valorValido(float valor) {
  return !isnan(valor);
}

// -------------------------------------------------------------------
static void atualizarEstado(float tInt, float hInt, float tExt, float hExt) {
  bool internoOk = valorValido(tInt) && valorValido(hInt);
  bool externoOk = valorValido(tExt) && valorValido(hExt);

  if (internoOk) {
    leituras.temperaturaInterna = tInt;
    leituras.umidadeInterna = hInt;
    leituras.internoValido = true;
  } else {
    leituras.internoValido = false;
  }

  if (externoOk) {
    leituras.temperaturaExterna = tExt;
    leituras.umidadeExterna = hExt;
    leituras.externoValido = true;
  } else {
    leituras.externoValido = false;
  }

  leituras.leituraValida = (leituras.internoValido || leituras.externoValido);
  leituras.timestampMs = millis();
}

// -------------------------------------------------------------------
void initDHT() {
  dhtInterno.begin();
  dhtExterno.begin();

  Serial.println("=== INIT DHT ===");
  Serial.print("DHT interno no pino ");
  Serial.println(DHT_PIN_INTERNO);
  Serial.print("DHT externo no pino ");
  Serial.println(DHT_PIN_EXTERNO);
}

// -------------------------------------------------------------------
void updateDHT() {
  if (millis() - ultimoUpdate < intervaloLeituraMs) {
    return;
  }

  ultimoUpdate = millis();

  float tInt = dhtInterno.readTemperature();
  float hInt = dhtInterno.readHumidity();

  float tExt = dhtExterno.readTemperature();
  float hExt = dhtExterno.readHumidity();

  atualizarEstado(tInt, hInt, tExt, hExt);
}

// -------------------------------------------------------------------
DHTReadings getDHTReadings() {
  return leituras;
}

float getTemperaturaInterna() { return leituras.temperaturaInterna; }
float getUmidadeInterna() { return leituras.umidadeInterna; }
float getTemperaturaExterna() { return leituras.temperaturaExterna; }
float getUmidadeExterna() { return leituras.umidadeExterna; }

bool isDHTInternoValido() { return leituras.internoValido; }
bool isDHTExternoValido() { return leituras.externoValido; }
bool isDHTLeituraValida() { return leituras.leituraValida; }