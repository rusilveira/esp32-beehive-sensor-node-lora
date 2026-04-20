#include <Arduino.h>
#include <SPI.h>
#include <RadioLib.h>
#include "loadcell_manager.h"
#include "dht_manager.h"
#include "display_manager.h"
#include "esp_sleep.h"

// ==========================
// PINOS LoRa (VALIDADOS)
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
// PROTOCOLO LoRa (VALIDADO)
// ==========================
constexpr uint8_t MAGIC_BYTE    = 0xA5;
constexpr uint8_t PROTO_VERSION = 0x01;

constexpr uint8_t MSG_DATA = 0x10;
constexpr uint8_t MSG_ACK  = 0x20;

constexpr uint8_t NODE_ID    = 0x01;
constexpr uint8_t GATEWAY_ID = 0xF0;

// ==========================
// CONFIG LoRa (VALIDADA)
// ==========================
constexpr float LORA_FREQ = 915.0;
constexpr float LORA_BW   = 500.0;
constexpr uint8_t LORA_SF = 7;
constexpr uint8_t LORA_CR = 5;
constexpr int8_t LORA_PWR = 2;
constexpr uint16_t LORA_PREAMBLE = 12;

constexpr uint16_t ACK_TIMEOUT = 800;
constexpr uint8_t MAX_RETRIES  = 3;

// Mantém sequência entre ciclos de deep sleep
RTC_DATA_ATTR uint8_t seq = 0;

// ==========================
// CONFIG DO CICLO DO NÓ
// ==========================
// Campo:
static const bool DEEP_SLEEP_ATIVO = true;
static const bool EXIGIR_START_SERIAL = false;
static const bool MODO_DEBUG_SERIAL = false;

// Bancada:
// static const bool DEEP_SLEEP_ATIVO = false;
// static const bool EXIGIR_START_SERIAL = true;
// static const bool MODO_DEBUG_SERIAL = true;

static const bool OLED_ATIVO_MODO_CAMPO = false; 
static const uint64_t TEMPO_SLEEP_US = 60ULL * 1000000ULL; // 1 minuto
static const unsigned long TEMPO_ESTABILIZACAO_MS = 60000;
static const unsigned long INTERVALO_RELATORIO_MS = 2000;

// ==========================
enum EstadoSistema {
  ESTADO_IDLE,
  ESTADO_RUNNING,
  ESTADO_FINALIZADO
};

static EstadoSistema estadoAtual = ESTADO_IDLE;
static unsigned long inicioCicloMs = 0;
static unsigned long ultimoRelatorioMs = 0;

// ==========================
// SNAPSHOT FINAL
// ==========================
struct SnapshotLeitura {
  float pesoBrutoKg;
  float pesoFiltradoKg;
  float temperaturaInterna;
  float umidadeInterna;
  float temperaturaExterna;
  float umidadeExterna;
  bool dhtInternoValido;
  bool dhtExternoValido;
};

// ==========================
// PAYLOAD REAL DA COLMEIA
// ==========================
struct PayloadColmeia {
  int16_t tempInterna_x100;
  uint16_t umidInterna_x100;

  int16_t tempExterna_x100;
  uint16_t umidExterna_x100;

  int32_t peso_x1000;

  uint8_t flags;
  uint8_t reservado;
};

// ==========================
// PROTÓTIPOS
// ==========================
static void printWakeupReason();
static void printSystemReport();
static void iniciarCiclo();
static void enterDeepSleep();
static void processSerialCommands();
static SnapshotLeitura capturarSnapshotFinal();
static PayloadColmeia montarPayload(const SnapshotLeitura& snap);
static void processarLeituraFinalUnica();

static uint16_t crc16(const uint8_t* data, size_t len);
static size_t buildPacket(uint8_t seqLocal, uint8_t* buf, PayloadColmeia& p);
static bool validAck(uint8_t* buf, size_t len, uint8_t seqLocal);
static void initRadio();
static bool sendPacketReliable(PayloadColmeia& payload);

// ==========================
// CRC16
// ==========================
static uint16_t crc16(const uint8_t* data, size_t len) {
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
static size_t buildPacket(uint8_t seqLocal, uint8_t* buf, PayloadColmeia& p) {
  size_t i = 0;

  buf[i++] = MAGIC_BYTE;
  buf[i++] = PROTO_VERSION;
  buf[i++] = MSG_DATA;
  buf[i++] = NODE_ID;
  buf[i++] = seqLocal;
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
static bool validAck(uint8_t* buf, size_t len, uint8_t seqLocal) {
  if (len < 8) return false;

  uint16_t crc_rx = (buf[len - 2] << 8) | buf[len - 1];
  uint16_t crc_calc = crc16(buf, len - 2);

  return (
    crc_rx == crc_calc &&
    buf[0] == MAGIC_BYTE &&
    buf[1] == PROTO_VERSION &&
    buf[2] == MSG_ACK &&
    buf[3] == GATEWAY_ID &&
    buf[4] == seqLocal &&
    buf[5] == 0x00
  );
}

// ==========================
// INIT RADIO
// ==========================
static void initRadio() {
  spiLoRa.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  delay(100);

  if (radio.begin() != RADIOLIB_ERR_NONE) {
    Serial.println("Erro init radio");
    while (true) {
      delay(1000);
    }
  }

  radio.setFrequency(LORA_FREQ);
  radio.setBandwidth(LORA_BW);
  radio.setSpreadingFactor(LORA_SF);
  radio.setCodingRate(LORA_CR);
  radio.setOutputPower(LORA_PWR);
  radio.setPreambleLength(LORA_PREAMBLE);

  Serial.println("Radio TX OK");
}

// ==========================
// ENVIO CONFIÁVEL
// ==========================
static bool sendPacketReliable(PayloadColmeia& payload) {
  uint8_t buf[64];
  size_t len = buildPacket(seq, buf, payload);

  for (int i = 0; i <= MAX_RETRIES; i++) {
    Serial.print("[LORA] tentativa ");
    Serial.println(i + 1);

    if (radio.transmit(buf, len) != RADIOLIB_ERR_NONE) {
      Serial.println("[LORA] transmit falhou");
      continue;
    }

    radio.startReceive();
    delay(10);

    uint8_t ack[16];
    int state = radio.receive(ack, sizeof(ack), ACK_TIMEOUT);

    if (state == RADIOLIB_ERR_NONE) {
      size_t l = radio.getPacketLength();

      if (validAck(ack, l, seq)) {
        Serial.print("[LORA] OK seq=");
        Serial.println(seq);
        seq++;
        return true;
      }
    }
  }

  Serial.print("[LORA] FAIL seq=");
  Serial.println(seq);
  return false;
}

// ==========================
// MONTA PAYLOAD
// ==========================
static PayloadColmeia montarPayload(const SnapshotLeitura& snap) {
  PayloadColmeia p{};

  if (snap.dhtInternoValido) {
    p.tempInterna_x100 = (int16_t)(snap.temperaturaInterna * 100.0f);
    p.umidInterna_x100 = (uint16_t)(snap.umidadeInterna * 100.0f);
    p.flags |= (1 << 0);
  }

  if (snap.dhtExternoValido) {
    p.tempExterna_x100 = (int16_t)(snap.temperaturaExterna * 100.0f);
    p.umidExterna_x100 = (uint16_t)(snap.umidadeExterna * 100.0f);
    p.flags |= (1 << 1);
  }

  p.peso_x1000 = (int32_t)(snap.pesoFiltradoKg * 1000.0f);
  p.reservado = 0;

  return p;
}

// ==========================
// CAPTURA SNAPSHOT FINAL
// ==========================
static SnapshotLeitura capturarSnapshotFinal() {
  DHTReadings dht = getDHTReadings();

  SnapshotLeitura snap;
  snap.pesoBrutoKg = getPesoBrutoKg();
  snap.pesoFiltradoKg = getPesoFiltradoKg();
  snap.temperaturaInterna = dht.temperaturaInterna;
  snap.umidadeInterna = dht.umidadeInterna;
  snap.temperaturaExterna = dht.temperaturaExterna;
  snap.umidadeExterna = dht.umidadeExterna;
  snap.dhtInternoValido = dht.internoValido;
  snap.dhtExternoValido = dht.externoValido;

  return snap;
}

// ==========================
// PROCESSA LEITURA FINAL
// ==========================
static void processarLeituraFinalUnica() {
  SnapshotLeitura snap = capturarSnapshotFinal();

  Serial.println();
  Serial.println(">>> LEITURA FINAL UNICA DO CICLO");

  Serial.print("Peso filtrado (kg): ");
  Serial.println(snap.pesoFiltradoKg, 3);

  Serial.print("Temp. interna (C): ");
  if (snap.dhtInternoValido) Serial.println(snap.temperaturaInterna, 1);
  else Serial.println("ERRO");

  Serial.print("Umid. interna (%): ");
  if (snap.dhtInternoValido) Serial.println(snap.umidadeInterna, 1);
  else Serial.println("ERRO");

  Serial.print("Temp. externa (C): ");
  if (snap.dhtExternoValido) Serial.println(snap.temperaturaExterna, 1);
  else Serial.println("ERRO");

  Serial.print("Umid. externa (%): ");
  if (snap.dhtExternoValido) Serial.println(snap.umidadeExterna, 1);
  else Serial.println("ERRO");

  PayloadColmeia payload = montarPayload(snap);

  // Após capturar o último valor, desliga o HX711 para reduzir interferência
  // e consumo antes da transmissão.
  Serial.println("[HX711] Power down antes do envio LoRa");
  powerDownLoadCell();
  delay(100);

  Serial.println("[LORA] Enviando...");
  bool ok = sendPacketReliable(payload);

  Serial.print("[LORA] Resultado: ");
  Serial.println(ok ? "OK" : "FALHA");
}

// ==========================
// WAKEUP REASON
// ==========================
static void printWakeupReason() {
  esp_sleep_wakeup_cause_t motivo = esp_sleep_get_wakeup_cause();

  Serial.print("Wakeup cause: ");

  switch (motivo) {
    case ESP_SLEEP_WAKEUP_TIMER:
      Serial.println("TIMER");
      break;
    case ESP_SLEEP_WAKEUP_UNDEFINED:
      Serial.println("POWER ON / RESET");
      break;
    default:
      Serial.print("OUTRO (");
      Serial.print((int)motivo);
      Serial.println(")");
      break;
  }
}

// ==========================
// COMANDOS SERIAL
// ==========================
static void processSerialCommands() {
  if (!Serial.available()) return;

  String comando = Serial.readStringUntil('\n');
  comando.trim();

  if (comando.length() == 0) return;

  if (comando.equalsIgnoreCase("start")) {
    if (estadoAtual == ESTADO_IDLE || estadoAtual == ESTADO_FINALIZADO) {
      iniciarCiclo();
    } else {
      Serial.println("Ciclo ja esta em execucao.");
    }
    return;
  }

  if (comando.equalsIgnoreCase("help")) {
    Serial.println();
    Serial.println("Comandos principais:");
    Serial.println("start = iniciar ciclo de leitura");
    Serial.println("help  = mostrar comandos");
    Serial.println("Comandos HX711: t = tara | c = calibrar | s = status | p = pausa");
    Serial.println();
    return;
  }

  processLoadCellCommand(comando);
}

// ==========================
// RELATÓRIO DE STATUS
// ==========================
static void printSystemReport() {
  DHTReadings dht = getDHTReadings();

  Serial.println();
  Serial.println("=========== STATUS SISTEMA ===========");

  Serial.print("Peso bruto (kg)    : ");
  Serial.println(getPesoBrutoKg(), 3);

  Serial.print("Peso filtrado (kg) : ");
  Serial.println(getPesoFiltradoKg(), 3);

  Serial.println("--------------------------------------");

  Serial.print("Temp. interna (C)  : ");
  if (dht.internoValido) Serial.println(dht.temperaturaInterna, 1);
  else Serial.println("ERRO");

  Serial.print("Umid. interna (%)  : ");
  if (dht.internoValido) Serial.println(dht.umidadeInterna, 1);
  else Serial.println("ERRO");

  Serial.print("Temp. externa (C)  : ");
  if (dht.externoValido) Serial.println(dht.temperaturaExterna, 1);
  else Serial.println("ERRO");

  Serial.print("Umid. externa (%)  : ");
  if (dht.externoValido) Serial.println(dht.umidadeExterna, 1);
  else Serial.println("ERRO");

  Serial.println("======================================");
}

// ==========================
// INICIA CICLO
// ==========================
static void iniciarCiclo() {
  inicioCicloMs = millis();
  ultimoRelatorioMs = 0;
  estadoAtual = ESTADO_RUNNING;

  // Garante HX711 ativo no início de cada ciclo
  powerUpLoadCell();

  Serial.println();
  Serial.println(">>> CICLO INICIADO");
  Serial.println("Sistema em estabilizacao e aquisicao...");
}

// ==========================
// DEEP SLEEP
// ==========================
static void enterDeepSleep() {
  Serial.println();
  Serial.println("Entrando em deep sleep...");
  Serial.flush();

  turnOffDisplay();
  delay(100);

  powerDownLoadCell();
  delay(50);

  esp_sleep_enable_timer_wakeup(TEMPO_SLEEP_US);
  esp_deep_sleep_start();
}

// ==========================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("==================================");
  Serial.println("Iniciando Colmeia Inteligente...");
  Serial.println("==================================");

  printWakeupReason();

  initLoadCell();
  initDHT();
  initDisplay();
  initRadio();

  if (!MODO_DEBUG_SERIAL && !OLED_ATIVO_MODO_CAMPO) {
    turnOffDisplay();
  }

  Serial.println();
  Serial.print("Deep sleep ativo: ");
  Serial.println(DEEP_SLEEP_ATIVO ? "SIM" : "NAO");

  Serial.print("Exigir start via serial: ");
  Serial.println(EXIGIR_START_SERIAL ? "SIM" : "NAO");

  if (EXIGIR_START_SERIAL) {
    Serial.println();
    Serial.println("Sistema em espera.");
    Serial.println("Digite 'start' para iniciar o ciclo.");
    estadoAtual = ESTADO_IDLE;
  } else {
    iniciarCiclo();
  }
}

// ==========================
void loop() {
  processSerialCommands();
  updateLoadCell();

  if (estadoAtual != ESTADO_RUNNING) {
    return;
  }

  if (isLoadCellPaused() || isLoadCellCalibrationWaiting()) {
    return;
  }

  updateDHT();

  DHTReadings dht = getDHTReadings();
  float pesoAtualKg = getPesoFiltradoKg();

  if (MODO_DEBUG_SERIAL || OLED_ATIVO_MODO_CAMPO) {
    turnOnDisplay();
    updateDisplay(pesoAtualKg, dht);
  }

  if (MODO_DEBUG_SERIAL && millis() - ultimoRelatorioMs >= INTERVALO_RELATORIO_MS) {
    ultimoRelatorioMs = millis();
    printSystemReport();
  }

  if (millis() - inicioCicloMs >= TEMPO_ESTABILIZACAO_MS) {
    estadoAtual = ESTADO_FINALIZADO;

    processarLeituraFinalUnica();

    if (DEEP_SLEEP_ATIVO) {
      enterDeepSleep();
    } else {
      Serial.println();
      Serial.println("Ciclo finalizado. Deep sleep desativado.");
      Serial.println("Digite 'start' para iniciar novo ciclo.");
    }
  }
}