#ifndef CONFIG_H
#define CONFIG_H

// Wi-Fi
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Backend
#define API_URL "http://YOUR_BACKEND_URL/api"

// Identificação da colmeia
#define COLMEIA_ID "jatai_01"

// Intervalo de envio para testes
#define ENVIO_INTERVALO_MS 10000

// =========================
// PINOS DO HARDWARE
// =========================

// HX711
// IMPORTANTE: sem conflito com LoRa
#define HX_DOUT 25
#define HX_SCK  4

// DHT22
#define DHT_PIN_INTERNO 13
#define DHT_PIN_EXTERNO 17
#define DHT_TYPE DHT22

// OLED SSD1306
#define OLED_SDA 21
#define OLED_SCL 22
#define OLED_RST 16
#define OLED_ADDRESS 0x3C

#endif