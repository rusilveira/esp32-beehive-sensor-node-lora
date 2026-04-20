#include "display_manager.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

static Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RST);

static unsigned long ultimoUpdateDisplay = 0;
static const unsigned long intervaloDisplayMs = 500;

// -------------------------------------------------------------------
static void printValorOuErro(float valor, bool valido, const char* sufixo, uint8_t casas = 1) {
  if (valido && !isnan(valor)) {
    display.print(valor, casas);
    display.print(sufixo);
  } else {
    display.print("--");
    display.print(sufixo);
  }
}

// -------------------------------------------------------------------
void initDisplay() {
  Wire.begin(OLED_SDA, OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("[OLED] Falha ao inicializar SSD1306");
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Colmeia Inteligente");
  display.println("Inicializando...");
  display.display();

  Serial.println("=== INIT OLED ===");
  Serial.println("Display SSD1306 inicializado com sucesso.");
}

// -------------------------------------------------------------------
void turnOffDisplay() {
  display.clearDisplay();
  display.display();
  display.ssd1306_command(SSD1306_DISPLAYOFF);
}

void turnOnDisplay() {
  display.ssd1306_command(SSD1306_DISPLAYON);
}

// -------------------------------------------------------------------
void updateDisplay(float pesoKg, const DHTReadings& dht) {
  if (millis() - ultimoUpdateDisplay < intervaloDisplayMs) {
    return;
  }

  ultimoUpdateDisplay = millis();

  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("COLMEIA - STATUS");

  display.setCursor(0, 12);
  display.print("Peso: ");
  display.print(pesoKg, 2);
  display.println(" kg");

  display.setCursor(0, 24);
  display.print("Int: ");
  printValorOuErro(dht.temperaturaInterna, dht.internoValido, "C ", 1);
  printValorOuErro(dht.umidadeInterna, dht.internoValido, "%", 0);

  display.setCursor(0, 36);
  display.print("Ext: ");
  printValorOuErro(dht.temperaturaExterna, dht.externoValido, "C ", 1);
  printValorOuErro(dht.umidadeExterna, dht.externoValido, "%", 0);

  display.setCursor(0, 52);
  display.print(dht.leituraValida ? "Sensores OK" : "Falha DHT");

  display.display();
}