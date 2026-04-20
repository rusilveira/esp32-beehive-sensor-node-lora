#include "loadcell_manager.h"
#include <HX711.h>
#include <Preferences.h>
#include <math.h>
#include "config.h"

HX711 scale;
Preferences preferences;

// ===== Configuracoes =====
static long offsetZero = 0;
static float fator = 30000.0;

// Filtro adaptativo
static float pesoFiltrado = 0.0;
static const float alphaRapido = 0.35f;
static const float alphaLento  = 0.12f;
static const float limiarMudanca = 0.05f;

// Controle
static bool aguardandoCalibracao = false;
static bool pausaLeitura = false;

static float pesoBrutoAtual = 0.0f;
static float pesoFiltradoAtual = 0.0f;

static unsigned long ultimoPrint = 0;
static const unsigned long intervaloPrintMs = 700;

// -------------------------------------------------------------------
static void saveCalibration() {
  preferences.begin("loadcell", false);
  preferences.putLong("offset", offsetZero);
  preferences.putFloat("factor", fator);
  preferences.end();
}

// -------------------------------------------------------------------
static void loadCalibration() {
  preferences.begin("loadcell", true);
  offsetZero = preferences.getLong("offset", 0);
  fator = preferences.getFloat("factor", 30000.0f);
  preferences.end();
}

// -------------------------------------------------------------------
static long mediaFiltrada(int n) {
  const int maxAmostras = 40;
  long valores[maxAmostras];

  if (n > maxAmostras) n = maxAmostras;
  if (n < 3) n = 3;

  for (int i = 0; i < n; i++) {
    while (!scale.is_ready()) {
      delay(1);
    }
    valores[i] = scale.read();
    delay(8);
  }

  for (int i = 0; i < n - 1; i++) {
    for (int j = i + 1; j < n; j++) {
      if (valores[j] < valores[i]) {
        long temp = valores[i];
        valores[i] = valores[j];
        valores[j] = temp;
      }
    }
  }

  int descartar = n / 5;
  int inicio = descartar;
  int fim = n - descartar;

  long soma = 0;
  int cont = 0;

  for (int i = inicio; i < fim; i++) {
    soma += valores[i];
    cont++;
  }

  return (cont > 0) ? soma / cont : 0;
}

// -------------------------------------------------------------------
static float applyAdaptiveFilter(float pesoBruto) {
  float diferenca = fabs(pesoBruto - pesoFiltrado);
  float alpha = (diferenca > limiarMudanca) ? alphaRapido : alphaLento;

  pesoFiltrado = alpha * pesoBruto + (1.0f - alpha) * pesoFiltrado;

  if (pesoFiltrado < 0.0f && pesoFiltrado > -0.03f) {
    pesoFiltrado = 0.0f;
  }

  if (pesoFiltrado > 0.0f && pesoFiltrado < 0.01f) {
    pesoFiltrado = 0.0f;
  }

  return pesoFiltrado;
}

// -------------------------------------------------------------------
static float lerPesoBrutoKg() {
  long leitura = mediaFiltrada(8);
  float peso = (offsetZero - leitura) / fator;

  if (peso < 0.0f && peso > -0.08f) {
    peso = 0.0f;
  }

  if (peso > 0.0f && peso < 0.03f) {
    peso = 0.0f;
  }

  return peso;
}

// -------------------------------------------------------------------
void tareLoadCell() {
  pausaLeitura = true;

  Serial.println();
  Serial.println("=== TARA HX711 ===");
  Serial.println("Deixe a base sem carga adicional...");
  delay(4000);

  offsetZero = mediaFiltrada(25);
  pesoFiltrado = 0.0f;
  saveCalibration();

  Serial.print("Novo OFFSET_ZERO = ");
  Serial.println(offsetZero);
  Serial.println("Tara concluida.");
  Serial.println();

  pausaLeitura = false;
}

// -------------------------------------------------------------------
void startCalibrationMode() {
  pausaLeitura = true;

  Serial.println();
  Serial.println("=== CALIBRACAO HX711 ===");
  Serial.println("Etapa 1: remova qualquer carga da base.");
  Serial.println("Fazendo tara automatica...");
  delay(3000);

  offsetZero = mediaFiltrada(25);
  pesoFiltrado = 0.0f;
  pesoBrutoAtual = 0.0f;
  pesoFiltradoAtual = 0.0f;
  saveCalibration();

  Serial.print("Novo OFFSET_ZERO = ");
  Serial.println(offsetZero);
  Serial.println("Tara concluida.");
  Serial.println();
  Serial.println("Etapa 2: coloque o peso conhecido na base.");
  Serial.println("Etapa 3: espere estabilizar e digite no serial o valor em kg.");
  Serial.println("Exemplo: 0.300 ou 1.000");
  Serial.println();

  aguardandoCalibracao = true;
}

// -------------------------------------------------------------------
static void showStatus() {
  Serial.println();
  Serial.println("=== STATUS HX711 ===");
  Serial.print("OFFSET_ZERO = ");
  Serial.println(offsetZero);
  Serial.print("FATOR = ");
  Serial.println(fator, 3);
  Serial.print("Peso bruto atual (kg) = ");
  Serial.println(pesoBrutoAtual, 3);
  Serial.print("Peso filtrado atual (kg) = ");
  Serial.println(pesoFiltradoAtual, 3);
  Serial.println();
}

// -------------------------------------------------------------------
void processLoadCellCommand(const String& entradaOriginal) {
  String entrada = entradaOriginal;
  entrada.trim();

  if (entrada.length() == 0) return;

  if (!aguardandoCalibracao) {
    if (entrada.equalsIgnoreCase("t")) {
      tareLoadCell();
      return;
    }

    if (entrada.equalsIgnoreCase("c")) {
      startCalibrationMode();
      return;
    }

    if (entrada.equalsIgnoreCase("s")) {
      showStatus();
      return;
    }

    if (entrada.equalsIgnoreCase("p")) {
      pausaLeitura = !pausaLeitura;
      Serial.print("Pausa leitura = ");
      Serial.println(pausaLeitura ? "SIM" : "NAO");
      return;
    }

    return;
  }

  float pesoConhecidoKg = entrada.toFloat();

  if (pesoConhecidoKg <= 0.0f) {
    Serial.println("Valor invalido. Digite um peso em kg, ex: 0.300");
    return;
  }

  long leituraComPeso = mediaFiltrada(25);
  long delta = offsetZero - leituraComPeso;

  if (delta <= 0) {
    Serial.println("Delta invalido para calibracao. Verifique peso e montagem.");
    aguardandoCalibracao = false;
    pausaLeitura = false;
    return;
  }

  fator = (float)delta / pesoConhecidoKg;
  pesoFiltrado = 0.0f;
  pesoBrutoAtual = 0.0f;
  pesoFiltradoAtual = 0.0f;
  saveCalibration();

  Serial.println();
  Serial.println("=== RESULTADO DA CALIBRACAO HX711 ===");
  Serial.print("Leitura com peso = ");
  Serial.println(leituraComPeso);
  Serial.print("Delta = ");
  Serial.println(delta);
  Serial.print("Peso conhecido (kg) = ");
  Serial.println(pesoConhecidoKg, 3);
  Serial.print("Novo fator = ");
  Serial.println(fator, 3);
  Serial.println("Calibracao concluida e salva na memoria.");
  Serial.println();

  aguardandoCalibracao = false;
  pausaLeitura = false;
}

void processLoadCellSerial() {
  if (!Serial.available()) return;

  String entrada = Serial.readStringUntil('\n');
  processLoadCellCommand(entrada);
}

// -------------------------------------------------------------------
void initLoadCell() {
  scale.begin(HX_DOUT, HX_SCK);
  powerUpLoadCell();
  scale.set_gain(128);

  loadCalibration();

  Serial.println("=== INIT LOADCELL ===");
  Serial.print("Offset carregado = ");
  Serial.println(offsetZero);
  Serial.print("Fator carregado = ");
  Serial.println(fator, 3);

  if (offsetZero == 0) {
    Serial.println("Nenhuma tara salva. Fazendo tara inicial...");
    tareLoadCell();
  }

  ultimoPrint = millis();
}

// -------------------------------------------------------------------
void updateLoadCell() {
  if (pausaLeitura || aguardandoCalibracao) {
    return;
  }

 if (millis() - ultimoPrint >= intervaloPrintMs) {
  ultimoPrint = millis();

  pesoBrutoAtual = lerPesoBrutoKg();
  pesoFiltradoAtual = applyAdaptiveFilter(pesoBrutoAtual);
  
  if (fabs(pesoFiltradoAtual) < 0.02) { pesoFiltradoAtual = 0.0;}
  
 }
}

// -------------------------------------------------------------------
float getPesoBrutoKg() {
  return pesoBrutoAtual;
}

float getPesoFiltradoKg() {
  return pesoFiltradoAtual;
}

long getLoadCellOffset() {
  return offsetZero;
}

float getLoadCellFactor() {
  return fator;
}

bool isLoadCellCalibrationWaiting() {
  return aguardandoCalibracao;
}

bool isLoadCellPaused() {
  return pausaLeitura;
}

// -------------------------------------------------------------------
void powerDownLoadCell() {
  scale.power_down();
}

void powerUpLoadCell() {
  scale.power_up();
  delay(100);
}