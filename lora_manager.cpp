#include "lora_manager.h"
#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>

// ==========================
// PINOS LoRa (iguais ao projeto validado)
// ==========================
#define LORA_SCK   5
#define LORA_MISO  19
#define LORA_MOSI  27
#define LORA_CS    18
#define LORA_DIO0  26
#define LORA_RST   14

SPIClass spiLoRa(VSPI);
SX1276 radio = new Module(LORA_CS, LORA_DIO0, LORA_RST, RADIOLIB_NC, spiLoRa);

// ==========================
// PROTOCOLO (igual ao validado)
// ==========================
constexpr uint8_t MAGIC_BYTE    = 0xA5;
constexpr uint8_t PROTO_VERSION = 0x01;

constexpr uint8_t MSG_DATA = 0x10;
constexpr uint8_t MSG_ACK  = 0x20;

constexpr uint8_t NODE_ID    = 0x01;
constexpr uint8_t GATEWAY_ID = 0xF0;

// ==========================
// CONFIG LoRa (igual ao validado)
// ==========================
constexpr float LORA_FREQ = 915.0;
constexpr float LORA_BW   = 500.0;
constexpr uint8_t LORA_SF = 7;
constexpr uint8_t LORA_CR = 5;
constexpr int8_t LORA_PWR = 2;
constexpr uint16_t LORA_PREAMBLE = 12;

constexpr uint16_t ACK_TIMEOUT = 800;
constexpr uint8_t MAX_RETRIES  = 3;

// ==========================
// SEQ
// ==========================
RTC_DATA_ATTR uint8_t seqLoRa = 0;

// ==========================
// CRC16
// ==========================
uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
    }
  }
  return crc;
}

// ==========================
// PACOTE DATA
// ==========================
size_t buildPacket(uint8_t seq, uint8_t* buf, PayloadColmeia& p) {
  size_t i = 0;

  buf[i++] = MAGIC_BYTE;
  buf[i++] = PROTO_VERSION;
  buf[i++] = MSG_DATA;
  buf[i++] = NODE_ID;
  buf[i++] = seq;
  buf[i++] = sizeof(PayloadColmeia);

  memcpy(&buf[i], &p, sizeof(PayloadColmeia));
  i += sizeof(PayloadColmeia);

  uint16_t crc = crc16(buf, i);
  buf[i++] = crc >> 8;
  buf[i++] = crc & 0xFF;

  return i;
}

// ==========================
// VALIDA ACK
// ==========================
bool validAck(uint8_t* buf, size_t len, uint8_t seq) {
  if (len < 8) return false;

  uint16_t crc_rx = (buf[len - 2] << 8) | buf[len - 1];
  uint16_t crc_calc = crc16(buf, len - 2);

  return (
    crc_rx == crc_calc &&
    buf[0] == MAGIC_BYTE &&
    buf[1] == PROTO_VERSION &&
    buf[2] == MSG_ACK &&
    buf[3] == GATEWAY_ID &&
    buf[4] == seq &&
    buf[5] == 0x00
  );
}

// ==========================
// RADIO INIT
// ==========================
void initRadio() {
  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  delay(100);

  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println("[LORA] Erro init radio");
    while (true);
  }

  radio.setFrequency(LORA_FREQ);
  radio.setBandwidth(LORA_BW);
  radio.setSpreadingFactor(LORA_SF);
  radio.setCodingRate(LORA_CR);
  radio.setOutputPower(LORA_PWR);
  radio.setPreambleLength(LORA_PREAMBLE);

  Serial.println("[LORA] Radio TX OK");
}

// ==========================
// ENVIO - baseado 1:1 no TX validado
// ==========================
bool sendPacketReliable(const PayloadColmeia& payloadConst) {
  PayloadColmeia payload = payloadConst;   // cópia local mutável para manter assinatura igual ao fluxo
  uint8_t buf[64];
  size_t len = buildPacket(seqLoRa, buf, payload);

  for (int i = 0; i <= MAX_RETRIES; i++) {
    Serial.print("[LORA] Enviando pacote seq=");
    Serial.print(seqLoRa);
    Serial.print(" tentativa ");
    Serial.print(i + 1);
    Serial.print("/");
    Serial.println(MAX_RETRIES + 1);

    if (radio.transmit(buf, len) != RADIOLIB_ERR_NONE) {
      Serial.println("[LORA] Falha no transmit");
      continue;
    }

    radio.startReceive();
    delay(10);

    uint8_t ack[16];
    int state = radio.receive(ack, sizeof(ack), ACK_TIMEOUT);

    if (state == RADIOLIB_ERR_NONE) {
      size_t l = radio.getPacketLength();

      if (validAck(ack, l, seqLoRa)) {
        Serial.print("[LORA] ACK OK seq=");
        Serial.println(seqLoRa);
        seqLoRa++;
        return true;
      }
    }
  }

  Serial.print("[LORA] FAIL seq=");
  Serial.println(seqLoRa);
  return false;
}