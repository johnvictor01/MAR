#include <SFE_BMP180.h>
#include <Wire.h>
#include <EEPROM.h>
//d2 buzzer d3 reset altimeter  d4 led vermelho d5 led verde d7 skib d12 ledligado
SFE_BMP180 pressure;
#define PIN_SKIB 7
#define PIN_LED_RED 4
#define PIN_LED_GREEN 5
#define PIN_BUZZER 2
#define PIN_LED_WORKING 12
#define PIN_BUTTON_ALTIMETER 3
#define MAX_ALTITUDE_ADDRESS 0
#define BASE_PRESSURE_ADDRESS 2
#define WINDOW_SIZE 5
#define ASCENT_THRESHOLD 5.0  // Altitude em metros para considerar que o foguete começou a subir
#define DESCENT_DETECTION_THRESHOLD 5.0  // Diferença em metros para detectar descida
#define JUMP_ASCENT 3
#define JUMP_DESCENT 30
#define SKIB_DEACTIVATION_TIME 400  // Contagem para desativar o SKIB

double baseline;
double altitudePoints[1000];
int altitudeIndex = 0;
double maxAltitude = -999;
bool armado = true;
int jumpSubida = 3;
int jumpDescida = 30;
int contadorDeTempo = 0;
bool isDescending = false;
bool sensorConnected = false;
bool rocketHasStartedAscent = false;  // Flag para verificar se o foguete começou a subir

double alturasSuavizadas[3] = {0, 0, 0};
double altitudeReadings[WINDOW_SIZE] = {0};
int currentReadingIndex = 0;
int totalReadings = 0;

// constantes de tempo
// Intervalos em milissegundos
const unsigned long BLINK_INTERVAL = 1000; // ciclo completo de 1 segundo
const unsigned long BLINK_ON_TIME  = 100;  // LED fica ligado 100 ms
const unsigned long BEEP_INTERVAL  = 5000; // beep a cada 5 s
const unsigned long BEEP_ON_TIME   = 100;  // buzzer ligado 100 ms

// Estado do LED de trabalho
unsigned long previousBlinkMillis = 0;
bool ledWorkingState = LOW;

// ——— Configuração do buzzer periódico ———
const unsigned long BEEP_PERIOD    = 5000;  // intervalo entre beeps (ms)
const unsigned long BEEP_DURATION  =  500;  // duração de cada beep (ms)
const unsigned int  BEEP_FREQUENCY = 2000;  // frequência do tom (Hz)

unsigned long previousBeepTime = 0;
bool isBeeping = false;

// Estado do buzzer
unsigned long previousBeepMillis = 0;
bool buzzerState = HIGH; // buzzer inativo é HIGH


void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando sistema de altímetro");
  pinMode(PIN_LED_WORKING, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_SKIB, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_BUTTON_ALTIMETER, INPUT);
 

  sensorConnected = pressure.begin();
  if (!sensorConnected) {
    Serial.println("BMP180 não encontrado! Tentando reconectar...");
    ledVermelho(true);
    ledVerde(false);
  } else {
    Serial.println("BMP180 inicializado com sucesso.");
    baseline = readBaselineFromEEPROM();
    ledVerde(true);
    ledVermelho(false);
  }
}

void loop() {
 
 
  double P = getPressure();
  if (P != -1) {
    double rawAltitude = pressure.altitude(P, baseline);
    double smoothedAltitude = getSmoothedAltitude(rawAltitude);

    alturasSuavizadas[2] = alturasSuavizadas[1];
    alturasSuavizadas[1] = alturasSuavizadas[0];
    alturasSuavizadas[0] = smoothedAltitude;

    Serial.println(smoothedAltitude);

    // Verificar se o foguete começou a subir
    if (!rocketHasStartedAscent && smoothedAltitude >= ASCENT_THRESHOLD) {
      rocketHasStartedAscent = true;
      Serial.println("Foguete começou a subir! Ativando buzzer.");
    }

    int jump = isDescending ? jumpDescida : jumpSubida;

    if (altitudeIndex < 1000 && smoothedAltitude > 3.0) {
      if (altitudeIndex % jump == 0) {
        altitudePoints[altitudeIndex] = smoothedAltitude;
      }
      altitudeIndex++;
      if (smoothedAltitude > maxAltitude) {
        maxAltitude = smoothedAltitude;
      }
    }

    // Verificar se deve ativar o sistema de recuperação
    if (alturasSuavizadas[0] <= maxAltitude - DESCENT_DETECTION_THRESHOLD && 
        armado &&
        isAltitudeStable()) {
      activateRecoverySystem();
    }

    if (!armado) {
      contadorDeTempo++;
      if (contadorDeTempo >= SKIB_DEACTIVATION_TIME) {
        desativarSkib();
      }
    }
  }

  atualizarEstadoSensor();
  updateWorkingLED();
  
  // Somente atualiza o buzzer se o foguete já começou a subir
  if (rocketHasStartedAscent) {
    updateBuzzerPeriodic();
  }
  
  delay(50);
}

void atualizarEstadoSensor() {
  if (!sensorConnected) {
    reconectarSensor();
  } else {
    Serial.println("Sensor funcionando normalmente.");
  }
}

void reconectarSensor() {
  sensorConnected = pressure.begin();
  if (sensorConnected) {
    Serial.println("Sensor reconectado com sucesso.");
    ledVerde(true);
    ledVermelho(false);
    baseline = readBaselineFromEEPROM();
  } else {
    Serial.println("Falha ao reconectar o sensor.");
    ledVerde(false);
    ledVermelho(true);
  }
}

// Outras funções existentes permanecem inalteradas
double getSmoothedAltitude(double newAltitude) {
  altitudeReadings[currentReadingIndex] = newAltitude;
  currentReadingIndex = (currentReadingIndex + 1) % WINDOW_SIZE;

  double sum = 0;
  int count = totalReadings < WINDOW_SIZE ? totalReadings : WINDOW_SIZE;
  for (int i = 0; i < count; i++) {
    sum += altitudeReadings[i];
  }

  totalReadings++;
  return sum / count;
}

double getPressure() {
  char status;
  double T, P;  // T = temperatura, P = pressão
  
  // ETAPA 1: Iniciar medição de temperatura
  status = pressure.startTemperature();  // Inicia a medição de temperatura
  if (status == 0) return -1;            // Se houver erro, interrompe imediatamente
  
  delay(status);  // Aguarda o tempo necessário para completar a medição
  
  // ETAPA 2: Obter a temperatura
  status = pressure.getTemperature(T);   // Lê o valor da temperatura
  if (status == 0) return -1;            // Se houver erro, interrompe imediatamente
  
  // ETAPA 3: Iniciar medição de pressão
  status = pressure.startPressure(1);    // Inicia a medição de pressão (1 = resolução padrão)
  if (status == 0) return -1;            // Se houver erro, interrompe imediatamente
  
  delay(status);  // Aguarda o tempo necessário para completar a medição
  
  // ETAPA 4: Obter a pressão (usando a temperatura como parâmetro)
  status = pressure.getPressure(P, T);   // Lê o valor da pressão
  if (status == 0) return -1;            // Se houver erro, interrompe imediatamente
  
  return P;  // Retorna o valor da pressão se tudo funcionou corretamente
}

void ativaSkip() {
  digitalWrite(PIN_SKIB, HIGH);
  ledVermelho(true);
}

void desativarSkib() {
  digitalWrite(PIN_SKIB, LOW);
  ledVermelho(false);
}

// Salva a altitude máxima na EEPROM
void saveMaxAltitudeToEEPROM(short int maxAltitudeInt) {
  EEPROM.put(MAX_ALTITUDE_ADDRESS, maxAltitudeInt);
}

// Lê a altitude máxima da EEPROM
double readMaxAltitudeFromEEPROM() {
  short int maxAltitudeInt;
  EEPROM.get(MAX_ALTITUDE_ADDRESS, maxAltitudeInt);
  return maxAltitudeInt / 10.0;
}

double readBaselineFromEEPROM() {
  double savedBaseline;
  EEPROM.get(BASE_PRESSURE_ADDRESS, savedBaseline);
  return (savedBaseline > 0) ? savedBaseline : getPressure();
}

void saveBaselineToEEPROM(double pressure) {
  EEPROM.put(BASE_PRESSURE_ADDRESS, pressure);
}

void ledVerde(bool estado) {
  digitalWrite(PIN_LED_GREEN, estado ? HIGH : LOW);
}

void ledVermelho(bool estado) {
  digitalWrite(PIN_LED_RED, estado ? HIGH : LOW);
}

// Atualiza o LED "working" (pino 12) sem bloquear
void updateWorkingLED() {
  unsigned long currentMillis = millis();

  // Verifica onde estamos no ciclo de piscada
  if (currentMillis - previousBlinkMillis >= BLINK_INTERVAL) {
    // Inicia um novo ciclo
    previousBlinkMillis = currentMillis;
    ledWorkingState = HIGH;
    digitalWrite(PIN_LED_WORKING, ledWorkingState);
  }
  // Se já passou o tempo de ficar ligado, desliga
  else if (currentMillis - previousBlinkMillis >= BLINK_ON_TIME) {
    ledWorkingState = LOW;
    digitalWrite(PIN_LED_WORKING, ledWorkingState);
  }
}

// Dispara o buzzer em tom fixo sem travar o loop
void updateBuzzerPeriodic() {
  unsigned long now = millis();

  if (!isBeeping) {
    // hora de iniciar um novo beep?
    if (now - previousBeepTime >= BEEP_PERIOD) {
      tone(PIN_BUZZER, BEEP_FREQUENCY);  // começa o tom
      isBeeping = true;
      previousBeepTime = now;
    }
  }
  else {
    // já tocou o suficiente?
    if (now - previousBeepTime >= BEEP_DURATION) {
      noTone(PIN_BUZZER);               // para o tom
      isBeeping = false;
      // deixa previousBeepTime marcado para contar o próximo período
      previousBeepTime = now;
    }
  }
}

// Verifica se as leituras de altitude estão estáveis
bool isAltitudeStable() {
  return abs(alturasSuavizadas[0] - alturasSuavizadas[1]) < 5.0 &&
         abs(alturasSuavizadas[1] - alturasSuavizadas[2]) < 5.0;
}

// Ativa o sistema de recuperação
void activateRecoverySystem() {
  ativaSkip();
  armado = false;
  isDescending = true;
  saveMaxAltitudeToEEPROM((short int)(maxAltitude * 10));
  
  Serial.println("Sistema de recuperação ativado!");
  Serial.print("Altitude máxima registrada: ");
  Serial.println(maxAltitude);
}
