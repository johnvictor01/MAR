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
#define MAX_ALTITUDE_ADDRESS 0        // Altitude máxima (2 bytes - short int)
#define BASE_PRESSURE_ADDRESS 2       // Pressão base (4 bytes - double)
#define FLIGHT_DATA_START_ADDRESS 6   // Início dos dados de voo (após os 6 bytes usados)
#define MAX_FLIGHT_DATA_POINTS 250    // Total de pontos (EEPROM: 512 - 6 = 506 bytes / 2 = 253, arredondamos para 250)
#define ALTITUDE_SCALE_FACTOR 10.0    // Fator de escala: multiplicar por 10 para preservar 1 casa decimal
#define WINDOW_SIZE 5
#define ASCENT_THRESHOLD 5.0  // Altitude em metros para considerar que o foguete começou a subir
#define DESCENT_DETECTION_THRESHOLD 5.0  // Diferença em metros para detectar descida
#define JUMP_ASCENT 3
#define JUMP_DESCENT 30
#define SKIB_DEACTIVATION_TIME 2000     // Contagem para desativar o SKIB
#define SKIB_BUZZER_DURATION 4000       // Duração do buzzer durante ativação do SKIB (4 segundos)
#define MIN_ALTITUDE_TO_RECORD 5.0      // Altitude mínima para iniciar gravação na EEPROM

// Intervalos para gravação na EEPROM
#define RECORD_INTERVAL_ASCENT 50     // 50ms durante a subida (maior frequência)
#define RECORD_INTERVAL_DESCENT 200   // 200ms durante a descida (menor frequência)

// Variáveis para gravação na EEPROM
unsigned int recordCounter = 0;        // Contador de registros gravados
unsigned long lastRecordTime = 0;      // Tempo da última gravação

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
bool apogeuDetectado = false; // Flag que indica se já detectamos o apogeu
double smoothedAltitude = 0;  // Adicionada como variável global

unsigned long skibActivatedAt = 0;
bool skibDeactivated  = false;

// Variáveis para controle do buzzer durante ativação do SKIB
bool skibBuzzerActive = false;
unsigned long skibBuzzerEndTime = 0;

double alturasSuavizadas[3] = {0, 0, 0};
double altitudeReadings[WINDOW_SIZE] = {0};
int currentReadingIndex = 0;
int totalReadings = 0;

// constantes de tempo
// Intervalos em milissegundos do led isWorking
const unsigned long BLINK_INTERVAL = 1000; // ciclo completo de 1 segundo
const unsigned long BLINK_ON_TIME  = 100;  // LED fica ligado 100 ms

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
  resetBaseline();

}


  
void loop() {
   
  double P = getPressure();
  if (P != -1) {
    double rawAltitude = pressure.altitude(P, baseline);  //  rawAltitude = altitude em metros daquele ponto
    smoothedAltitude = getSmoothedAltitude(rawAltitude); // vai pegar a media das ultimas 5 altitudes(suavizada)

    alturasSuavizadas[2] = alturasSuavizadas[1];  
    alturasSuavizadas[1] = alturasSuavizadas[0];
    alturasSuavizadas[0] = smoothedAltitude;

    Serial.println(smoothedAltitude);

    // Verificar se o foguete começou a subir
    // if (!rocketHasStartedAscent && smoothedAltitude >= ASCENT_THRESHOLD) { 
    //   rocketHasStartedAscent = true;
    //   Serial.println("Foguete começou a subir! Ativando buzzer.");
    // }

    int jump = isDescending ? jumpDescida : jumpSubida; //isdescending = false, vai saber se usa o jump de subida que pega mais dados, ou o de descida que pega menos dados
  //começa com a taxa de amostragem da subida 3 (salva na eeprom)


    //if de salvamento dos dados (tem 512bytes na eeprom)
    //perguntar pra john pq 1000(deveria ser cada byte da eeprom!)
    if (altitudeIndex < 1000 && smoothedAltitude > 3.0) {
      if (altitudeIndex % jump == 0) {
        altitudePoints[altitudeIndex] = smoothedAltitude;
      }//saber a taxa de amostragem se é de subida ou descida
      altitudeIndex++;//passa pro proximo byte da eeprom
      if (smoothedAltitude > maxAltitude) {
        maxAltitude = smoothedAltitude;
        //maxAltitude = rawAltitude //é importante saber o apogeu real e nao suavizado? no apogeu a velocidade é 0 entao poderia deixar suavizado
      }
    }

    // Escolher intervalo de gravação com base se está subindo ou descendo
    int currentRecordInterval = isDescending ? RECORD_INTERVAL_DESCENT : RECORD_INTERVAL_ASCENT;

    // Gravar altitude na EEPROM com intervalo adaptativo - APENAS SE ALTURA > 5m
    if (millis() - lastRecordTime >= currentRecordInterval && 
        recordCounter < MAX_FLIGHT_DATA_POINTS && 
        rawAltitude >= MIN_ALTITUDE_TO_RECORD) {
      
      saveAltitudeToEEPROM(rawAltitude, recordCounter);
      recordCounter++;
      lastRecordTime = millis();
      
      // Log mais detalhado
      Serial.print(isDescending ? "DESCIDA" : "SUBIDA");
      Serial.print(" - Ponto ");
      Serial.print(recordCounter);
      Serial.print(": ");
      Serial.print(rawAltitude);
      Serial.println("m");
    }

    // Verificar se deve ativar o sistema de recuperação ao começar a descer um limiar do apogeu 
    if (alturasSuavizadas[0] <= maxAltitude - DESCENT_DETECTION_THRESHOLD &&   
        armado &&
        isAltitudeStable()) {
      activateRecoverySystem();
      apogeuDetectado = true;
      isDescending = true;
    }

    if (!armado 
        && !skibDeactivated 
        && millis() - skibActivatedAt >= SKIB_DEACTIVATION_TIME) {
      desativarSkib();        // só vai rodar uma vez
      skibDeactivated = true; // garante que nunca mais entre aqui
      Serial.println("SKIB desativado após timeout.");
    }
  }

  // Gerenciar o buzzer durante a ativação do SKIB
  updateSkibBuzzer();
  
  atualizarEstadoSensor(); //
  updateWorkingLED(); //
  
  if (smoothedAltitude < 5.0 && !skibBuzzerActive) {
    updateBuzzerPeriodic();
  } else if (!skibBuzzerActive) {
    noTone(PIN_BUZZER);
    isBeeping = false;
    previousBeepTime = millis();
  }
  
  delay(50);
}

// Gerencia o buzzer durante a ativação do SKIB
void updateSkibBuzzer() {
  if (skibBuzzerActive) {
    // Se o tempo de buzzer do SKIB acabou
    if (millis() >= skibBuzzerEndTime) {
      noTone(PIN_BUZZER);
      skibBuzzerActive = false;
      Serial.println("Buzzer do SKIB finalizado.");
    }
    // Caso contrário, manter o buzzer ligado
  }
}

// Salva a altitude na EEPROM como um short int
void saveAltitudeToEEPROM(double altitude, unsigned int index) {
  // Converte altitude para short int com fator de escala
  short int altitudeScaled;
  
  // Limitação: tratamento para altitudes fora do intervalo representável
  if (altitude < -3276.8) {
    altitudeScaled = -32768;
  } else if (altitude > 3276.7) {
    altitudeScaled = 32767;
  } else {
    altitudeScaled = (short int)(altitude * ALTITUDE_SCALE_FACTOR);  // Multiplica por 10 para preservar uma casa decimal
  }
  
  // Calcula o endereço na EEPROM (cada registro ocupa 2 bytes)
  int address = FLIGHT_DATA_START_ADDRESS + (index * 2);
  
  // Grava o short int na EEPROM
  EEPROM.put(address, altitudeScaled);
}

// Lê a altitude da EEPROM
double readAltitudeFromEEPROM(unsigned int index) {
  int address = FLIGHT_DATA_START_ADDRESS + (index * 2);
  short int altitudeScaled;
  
  // Lê o short int da EEPROM
  EEPROM.get(address, altitudeScaled);
  
  // Converte de volta para double
  return altitudeScaled / ALTITUDE_SCALE_FACTOR;
}

void atualizarEstadoSensor() {
  if (!sensorConnected) {
    reconectarSensor();
  } else {
    Serial.println("Sensor funcionando normalmente.");
  }
}

void reconectarSensor() { //se o sensor ta funcionando o led verde ta aceso
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


double getSmoothedAltitude(double newAltitude) { //faz  a media das ultimas windowsize altitudes
  altitudeReadings[currentReadingIndex] = newAltitude; //o indice atual recebe a altura atual
  currentReadingIndex = (currentReadingIndex + 1) % WINDOW_SIZE; // contador de 1 a 5

  double sum = 0;
  int count = totalReadings < WINDOW_SIZE ? totalReadings : WINDOW_SIZE;  //if totalreadins(começa em 0), menor que o vetor window size 5
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
  unsigned long now = millis();

  // 1) Reseta o ciclo a cada BLINK_INTERVAL
  if (now - previousBlinkMillis >= BLINK_INTERVAL) {
    previousBlinkMillis = now;
  }

  // 2) Liga o LED durante os primeiros BLINK_ON_TIME ms do ciclo, caso contrário desliga
  if (now - previousBlinkMillis < BLINK_ON_TIME) {
    digitalWrite(PIN_LED_WORKING, HIGH);
  } else {
    digitalWrite(PIN_LED_WORKING, LOW);
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

void resetBaseline() {
  baseline = getPressure();
  saveBaselineToEEPROM(baseline);
  Serial.println("Baseline resetado.");
}


// Ativa o sistema de recuperação
void activateRecoverySystem() {
  ativaSkip();               // liga o SKIB
  armado = false;           
  isDescending = true;
  ledVermelho(HIGH);
  skibActivatedAt = millis();  // marca o instante de acionamento
  skibDeactivated  = false;    // garante que ainda não desativamos
  saveMaxAltitudeToEEPROM((short int)(maxAltitude * 10));
  
  // Ativa o buzzer por 4 segundos
  tone(PIN_BUZZER, BEEP_FREQUENCY);
  skibBuzzerActive = true;
  skibBuzzerEndTime = millis() + SKIB_BUZZER_DURATION;
  
  Serial.println("Sistema de recuperação ativado com buzzer!");
}
