#ifndef CONFIG_H
#define CONFIG_H

// =====================================================
// WI-FI
// =====================================================

#define WIFI_SSID     "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// =====================================================
// BACKEND
// =====================================================

#define API_URL "http://YOUR_BACKEND_URL/api"

// =====================================================
// IDENTIFICACAO DA COLMEIA
// =====================================================

#define COLMEIA_ID "jatai_01"

// =====================================================
// INTERVALO DE ENVIO
// =====================================================

#define ENVIO_INTERVALO_MS 10000

// =====================================================
// PINOS DO HARDWARE
// =====================================================

// HX711
#define HX_DOUT 25
#define HX_SCK  4

// DHT22
#define DHT_PIN_INTERNO 13
#define DHT_PIN_EXTERNO 17
#define DHT_TYPE        DHT22

// OLED SSD1306
#define OLED_SDA     21
#define OLED_SCL     22
#define OLED_RST     16
#define OLED_ADDRESS 0x3C

// Monitoramento da bateria
#define BAT_ADC_PIN    34
#define BAT_R1         200000.0f
#define BAT_R2         100000.0f
#define BAT_ADC_VREF   3.3f
#define BAT_CAL_FACTOR 1.123f

#endif