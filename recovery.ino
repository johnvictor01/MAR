#include <SFE_BMP180.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SD.h>
#include <SPI.h>

File logFile;

SFE_BMP180 pressure;
#define PIN_SKIB 1
#define PIN_BUZZER 2
#define PIN_LED_RED 6
#define PIN_LED_GREEN 5 //na placa do computador de bordo, está invertido com o blue, portanto depois mude esta linha pra 5 e a do blue para 3
#define PIN_LED_BLUE 3
#define PIN_SPI_CS 10
#define PIN_SPI_MOSI 11
#define PIN_SPI_MISO 12 
#define PIN_SPI_SCK 13


#define MAX_ALTITUDE_ADDRESS 0        // Altitude máxima (2 bytes - short int)
#define BASE_PRESSURE_ADDRESS 2       // Pressão base (4 bytes - double)
#define FLIGHT_DATA_START_ADDRESS 6   // Início dos dados de voo (após os 6 bytes usados)
//#define MAX_FLIGHT_DATA_POINTS 250    // Total de pontos (EEPROM: 512 - 6 = 506 bytes / 2 = 253, arredondamos para 250)
#define MAX_FLIGHT_DATA_POINTS 497    // Total de pontos (EEPROM: 1000 - 6 = 994 bytes / 2 = 497)
#define ALTITUDE_SCALE_FACTOR 10.0    // Fator de escala: multiplicar por 10 para preservar 1 casa decimal
#define WINDOW_SIZE 5
#define ASCENT_THRESHOLD 5.0  // Altitude em metros para considerar que o foguete começou a subir
#define DESCENT_DETECTION_THRESHOLD 5.0  // Diferença em metros para detectar descida
#define JUMP_ASCENT 3
#define JUMP_DESCENT 30
#define SKIB_DEACTIVATION_TIME 2000     // Contagem para desativar o SKIB
#define SKIB_BUZZER_DURATION 4000       // Duração do buzzer durante ativação do SKIB (4 segundos)
#define MIN_ALTITUDE_TO_RECORD 5.0      // Altitude mínima para iniciar gravação na EEPROM
#define PRE_FLIGHT_QUEUE_SIZE 4        // Tamanho da fila de pontos pré-voo

// Intervalos para gravação na EEPROM
#define RECORD_INTERVAL_ASCENT 50     // 50ms durante a subida (maior frequência)
#define RECORD_INTERVAL_DESCENT 200   // 200ms durante a descida (menor frequência)

// Intervalos para gravação no SD Card
#define RECORD_INTERVAL_SD 10     // 10ms para gravação no SD Card

// Estados do sistema
enum FlightState {
  PRE_FLIGHT,    // Pré-voo: monitorando altura, bipando, mas não gravando na EEPROM
  ASCENT,        // Ascensão: gravando dados na EEPROM com alta frequência
  RECOVERY       // Recuperação: SKIB ativado, gravando com baixa frequência
};

// Variáveis para gravação na EEPROM
unsigned int recordCounter = 0;        // Contador de registros gravados
unsigned long lastRecordTime = 0;      // Tempo da última gravação

// Variáveis para gravação no SD Card
unsigned long lastSdRecordTime = 0;      // Tempo da última gravação no SD

// Sistema de estados
FlightState currentState = PRE_FLIGHT;

// Fila circular para pontos pré-voo
double preFlightQueue[PRE_FLIGHT_QUEUE_SIZE];
int preFlightQueueIndex = 0;
bool preFlightQueueFull = false;


double baseline;
double maxAltitude = -999;
bool armado = true;
int jumpSubida = JUMP_ASCENT;
int jumpDescida = JUMP_DESCENT;
int contadorDeTempo = 0;
bool isDescending = false;
bool sensorConnected = false;
bool apogeuDetectado = false; // Flag que indica se já detectamos o apogeu
double smoothedAltitude = 0.0;  // Adicionada como variável global

unsigned long skibActivatedAt = 0;
bool skibDeactivated  = false;

// Variáveis para controle do buzzer durante ativação do SKIB
bool skibBuzzerActive = false;
unsigned long skibBuzzerEndTime = 0;


double alturasSuavizadas[5] = {0, 0, 0, 0, 0};
double altitudeReadings[WINDOW_SIZE] = {0}; // ? inicializando array com escalar?
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
const unsigned long BEEP_PERIOD_PREFLIGHT = 3000;  // intervalo entre beeps no pré-voo (ms)
const unsigned long BEEP_PERIOD_ASCENT     = 1000;  // intervalo entre beeps na ascensão (ms)
const unsigned long BEEP_PERIOD_RECOVERY   = 500;   // intervalo entre beeps na recuperação (ms)
const unsigned long BEEP_DURATION  =  400;  // duração de cada beep (ms)
const unsigned int  BEEP_FREQUENCY_PREFLIGHT = 1500;  // frequência para pré-voo (Hz)
const unsigned int  BEEP_FREQUENCY_ASCENT    = 2093;  // frequência para ascensão (Hz) 
const unsigned int  BEEP_FREQUENCY_RECOVERY  = 3136;  // frequência para recuperação (Hz)
const unsigned int  BEEP_FREQUENCY_SKIB      = 2637;  // frequência após ativação do skib (Hz)

unsigned long previousBeepTime = 0;
bool isBeeping = false;

// Estado do buzzer
unsigned long previousBeepMillis = 0;
bool buzzerState = HIGH; // buzzer inativo é HIGH

// Controle de tempo para leitura do sensor (substitui o delay)
const unsigned long SENSOR_READ_INTERVAL = 50;  // 50ms entre leituras do sensor
unsigned long lastSensorReadTime = 0;

// Armazena o começo do voo
unsigned long flightStartTime = 0; 

void setup() {
  Serial.begin(9600);
  Serial.println(F("Iniciando sistema de altímetro"));

  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_SKIB, OUTPUT);

  setRGB(0, 0, 255); // Blue light while booting

  bool sdReady = setupSDCard();
  bool sensorReady = pressure.begin();
  sensorConnected = sensorReady;

  if (sensorReady && sdReady) {
    Serial.println(F("Sistema pronto. Altimetro e SD Card OK."));
    setRGB(0, 255, 0); // Green
    delay(1000); // Show green light for 1 second
  } else {
    // Loop indefinitely and blink the error color
    while (1) {
      if (!sensorReady && !sdReady) {
        Serial.println(F("ERRO: Falha no Altimetro e no SD Card!"));
        setRGB(255, 0, 0); // Red
      } else if (sensorReady && !sdReady) {
        Serial.println(F("ERRO: Falha no SD Card!"));
        setRGB(255, 255, 0); // Yellow
      } else if (!sensorReady && sdReady) {
        Serial.println(F("ERRO: Falha no Altimetro!"));
        setRGB(255, 0, 255); // Magenta
      }
      delay(500);
      setRGB(0, 0, 0); // Off
      delay(500);
    }
  }

  baseline = readBaselineFromEEPROM();
  resetBaseline();
  
  // Inicializar fila pré-voo
  initializePreFlightQueue();
}
bool setupSDCard() {
  Serial.print(F("Initializing SD card..."));
  pinMode(PIN_SPI_CS, OUTPUT);

  if (!SD.begin(PIN_SPI_CS)) {
    Serial.println(F("initialization failed!"));
    return false;
  }
  Serial.println(F("initialization done."));

  // Create a new file
  char fileName[] = "LOG_00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    fileName[4] = i / 10 + '0';
    fileName[5] = i % 10 + '0';
    if (!SD.exists(fileName)) {
      // Create the file
      logFile = SD.open(fileName, FILE_WRITE);
      break;
    }
  }

  if (logFile) {
    Serial.print(F("Logging to: "));
    Serial.println(fileName);
    logFile.println("Timestamp,Altitude,State");
    logFile.flush();
    return true;
  } else {
    Serial.println(F("Couldn't create file!"));
    return false;
  }
}

void logDataToSD() {
  if (logFile) {

    unsigned long tempoDeVoo_ms = millis() - flightStartTime; //Converter o tempo de vôo para tempo de vôo efetivo em segundos 
    
    double tempoDeVoo_s = tempoDeVoo_ms / 1000.0;
    
    logFile.print(tempoDeVoo_s);
    
    logFile.print(",");
    logFile.print(smoothedAltitude);
    logFile.print(",");
    logFile.println(getStateString());
    logFile.flush();
  }
}


  
void loop() {
  // Controlar frequência de leitura do sensor sem bloquear o programa
  unsigned long currentTime = millis();

  // Grava no cartão SD em alta frequência
  if (currentTime - lastSdRecordTime >= RECORD_INTERVAL_SD) {
    lastSdRecordTime = currentTime;
    if (currentState != PRE_FLIGHT) { // Começa a gravar no SD depois do PRE_FLIGHT
        logDataToSD();
    }
  }
  
  // Só faz leitura do sensor a cada SENSOR_READ_INTERVAL ms
  if (currentTime - lastSensorReadTime >= SENSOR_READ_INTERVAL) {
    lastSensorReadTime = currentTime;
    
    double P = getPressure();
    if (P != -1) {
      double rawAltitude = pressure.altitude(P, baseline);
      smoothedAltitude = getSmoothedAltitude(rawAltitude);

      // Atualizar alturas suavizadas
      updateSmoothAltitudeHistory();


      if (currentState != PRE_FLIGHT) {
          logDataToSD(); // Chama a gravação da altura para o modulo sd
      }

      Serial.print(F("Estado: "));
      Serial.print(getStateString());
      Serial.print(F(", Altitude: "));
      Serial.println(smoothedAltitude);

      // Lógica baseada em estados
      switch (currentState) {
        case PRE_FLIGHT:
          handlePreFlightState();
          break;
        case ASCENT:
          handleAscentState();
          break;
        case RECOVERY:
          handleRecoveryState();
          break;
      }

      // Verificar transições de estado
      checkStateTransitions();

      // Atualizar altitude máxima
      if (smoothedAltitude > maxAltitude) {
        maxAltitude = smoothedAltitude;
      }

      // Gerenciar desativação do SKIB
      if (!armado && !skibDeactivated && millis() - skibActivatedAt >= SKIB_DEACTIVATION_TIME) {
        desativarSkib();
        skibDeactivated = true;
        Serial.println(F("SKIB desativado após timeout."));
      }
    }
  }

  // Gerenciar buzzer e LEDs (executam sempre, independente da leitura do sensor)
  updateSkibBuzzer();
  updateStateLED();
  updateBuzzerByState();
  
  // Removido o delay(50) - agora controlado por millis()
}

// Gerencia o buzzer durante a ativação do SKIB
void updateSkibBuzzer() {
  if (skibBuzzerActive) {
    // Se o tempo de buzzer do SKIB acabou
    if (millis() >= skibBuzzerEndTime) {
      noTone(PIN_BUZZER);
      skibBuzzerActive = false;
      Serial.println(F("Buzzer do SKIB finalizado."));
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
  }
  else if (altitude > 3276.7) {
    altitudeScaled = 32767;
  }
  else {
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

void ativaSkib() {
  digitalWrite(PIN_SKIB, HIGH);
  setRGB(255, 0, 0); // Red
}

void desativarSkib() {
  digitalWrite(PIN_SKIB, LOW);
  // The LED will be controlled by updateStateLED in the loop
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

void setRGB(byte red, byte green, byte blue) {
  analogWrite(PIN_LED_RED, red);
  analogWrite(PIN_LED_GREEN, green);
  analogWrite(PIN_LED_BLUE, blue);
}


// Dispara o buzzer baseado no estado atual do sistema
void updateBuzzerByState() {
  if (skibBuzzerActive) return; // Não interferir com o buzzer do SKIB
  
  unsigned long now = millis();
  unsigned long beepPeriod;
  unsigned int beepFrequency;
  
  // Definir período e frequência baseado no estado
  switch (currentState) {
    case PRE_FLIGHT:
      beepPeriod = BEEP_PERIOD_PREFLIGHT;
      beepFrequency = BEEP_FREQUENCY_PREFLIGHT;
      break;
    case ASCENT:
      beepPeriod = BEEP_PERIOD_ASCENT;
      beepFrequency = BEEP_FREQUENCY_ASCENT;
      break;
    case RECOVERY:
      if (armado) {
        beepPeriod = BEEP_PERIOD_RECOVERY;
        beepFrequency = BEEP_FREQUENCY_RECOVERY;
      } else {
        beepPeriod = BEEP_PERIOD_RECOVERY;
        beepFrequency = BEEP_FREQUENCY_SKIB;
      }
      break;
  }

  if (!isBeeping) {
    // Hora de iniciar um novo beep?
    if (now - previousBeepTime >= beepPeriod) {
      tone(PIN_BUZZER, beepFrequency);
      isBeeping = true;
      previousBeepTime = now;
    }
  } else {
    // Já tocou o suficiente?
    if (now - previousBeepTime >= BEEP_DURATION) {
      noTone(PIN_BUZZER);
      isBeeping = false;
      previousBeepTime = now;
    }
  }
}

void updateStateLED() {
  unsigned long currentTime = millis();
  int blinkSpeed;

  // Determine blink speed based on state
  switch (currentState) {
    case PRE_FLIGHT:
      blinkSpeed = 1000; // Slow blink
      break;
    case ASCENT:
      blinkSpeed = 250;  // Fast blink
      break;
    case RECOVERY:
      blinkSpeed = 1500; // Slower blink
      break;
    default:
      blinkSpeed = 1000;
      break;
  }

  // Simple blinking logic
  if ((currentTime / blinkSpeed) % 2 == 0) {
    // Set color based on state
    switch (currentState) {
      case PRE_FLIGHT:
        setRGB(0, 0, 255); // Blue
        break;
      case ASCENT:
        setRGB(0, 255, 255); // Cyan
        break;
      case RECOVERY:
        setRGB(255, 165, 0); // Orange
        break;
    }
  } else {
    // LED is off during the other half of the blink cycle
    setRGB(0, 0, 0);
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
  Serial.print(F("Baseline resetado: "));
  Serial.print(baseline);
  Serial.println(F(" m (- nível do mar)."));
}


// Ativa o sistema de recuperação
void activateRecoverySystem() {
  ativaSkib();               // liga o SKIB
  armado = false;           
  isDescending = true;
  currentState = RECOVERY;   // Transição para estado de recuperação
  setRGB(255, 0, 0); // Red
  skibActivatedAt = millis();  // marca o instante de acionamento
  skibDeactivated  = false;    // garante que ainda não desativamos
  saveMaxAltitudeToEEPROM((short int)(maxAltitude * 10));
  
  // Ativa o buzzer por 4 segundos
  tone(PIN_BUZZER, BEEP_FREQUENCY_RECOVERY);
  skibBuzzerActive = true;
  skibBuzzerEndTime = millis() + SKIB_BUZZER_DURATION;
  
  Serial.println(F("Sistema de recuperação ativado com buzzer!"));
}

// ========== NOVAS FUNÇÕES PARA SISTEMA DE ESTADOS ==========

// Inicializa a fila de pontos pré-voo
void initializePreFlightQueue() {
  for (int i = 0; i < PRE_FLIGHT_QUEUE_SIZE; i++) {
    preFlightQueue[i] = 0.0;
  }
  preFlightQueueIndex = 0;
  preFlightQueueFull = false;
}

// Adiciona um ponto à fila circular pré-voo
void addToPreFlightQueue(double altitude) {
  preFlightQueue[preFlightQueueIndex] = altitude;
  preFlightQueueIndex = (preFlightQueueIndex + 1) % PRE_FLIGHT_QUEUE_SIZE;
  if (preFlightQueueIndex == 0) {
    preFlightQueueFull = true;
  }
}

// Atualiza o histórico de alturas suavizadas
void updateSmoothAltitudeHistory() {
  alturasSuavizadas[4] = alturasSuavizadas[3];  
  alturasSuavizadas[3] = alturasSuavizadas[2];  
  alturasSuavizadas[2] = alturasSuavizadas[1];  
  alturasSuavizadas[1] = alturasSuavizadas[0];
  alturasSuavizadas[0] = smoothedAltitude;
}

// Retorna string do estado atual
String getStateString() {
  switch (currentState) {
    case PRE_FLIGHT: return "PRE_FLIGHT";
    case ASCENT: return "ASCENT";
    case RECOVERY: return "RECOVERY";
    default: return "UNKNOWN";
  }
}

// Gerencia o estado PRE_FLIGHT
void handlePreFlightState() {
  // Adicionar altitude atual à fila pré-voo
  addToPreFlightQueue(smoothedAltitude);
  
  // Não grava na EEPROM durante pré-voo, apenas monitora
  Serial.print(F("Pré-voo - Altitude: "));
  Serial.print(smoothedAltitude);
  Serial.println(F(" m (não gravando na EEPROM)"));
}

// Gerencia o estado ASCENT
void handleAscentState() {
  // Gravar altitude na EEPROM com alta frequência
  if (millis() - lastRecordTime >= RECORD_INTERVAL_ASCENT && 
      recordCounter < MAX_FLIGHT_DATA_POINTS) {
    
    saveAltitudeToEEPROM(smoothedAltitude, recordCounter);
    recordCounter++;
    lastRecordTime = millis();
    
    Serial.print(F("ASCENSÃO - Ponto: "));
    Serial.print(recordCounter);
    Serial.print(F(", Tempo: "));
    Serial.print(lastRecordTime);
    Serial.print(F(" ms, Altura: "));
    Serial.print(smoothedAltitude);
    Serial.println(F(" m"));
  }
}

// Gerencia o estado RECOVERY
void handleRecoveryState() {
  // Gravar altitude na EEPROM com baixa frequência
  if (millis() - lastRecordTime >= RECORD_INTERVAL_DESCENT && 
      recordCounter < MAX_FLIGHT_DATA_POINTS) {
    
    saveAltitudeToEEPROM(smoothedAltitude, recordCounter);
    recordCounter++;
    lastRecordTime = millis();
    
    Serial.print(F("RECUPERAÇÃO - Ponto: "));
    Serial.print(recordCounter);
    Serial.print(F(", Tempo: "));
    Serial.print(lastRecordTime);
    Serial.print(F(" ms, Altura: "));
    Serial.print(smoothedAltitude);
    Serial.println(F(" m"));
  }
}

// Verifica e executa transições de estado
void checkStateTransitions() {
  switch (currentState) {
    case PRE_FLIGHT:
      // Transição para ASCENT quando altura ultrapassa threshold
      if (smoothedAltitude >= ASCENT_THRESHOLD) {
        currentState = ASCENT;
        flightStartTime = millis(); //começa a contar tempo de voo

        Serial.println(F("TRANSIÇÃO: PRE_FLIGHT -> ASCENT"));
        
        // Salvar os pontos pré-voo na EEPROM
        savePreFlightPointsToEEPROM();
        
        // Inicializar tempo de gravação
        lastRecordTime = millis();
      }
      break;
      
    case ASCENT:
      // Transição para RECOVERY quando detecta descida significativa
      if (alturasSuavizadas[0] <= maxAltitude - DESCENT_DETECTION_THRESHOLD && armado) {
        Serial.println(F("TRANSIÇÃO: ASCENT -> RECOVERY"));
        activateRecoverySystem();
        apogeuDetectado = true;
        isDescending = true;
      }
      break;
      
    case RECOVERY:
      // Estado final - sem transições
      break;
  }
}

// Salva os pontos pré-voo na EEPROM
void savePreFlightPointsToEEPROM() {
  Serial.println(F("Salvando pontos pré-voo na EEPROM..."));
  
  // Determinar quantos pontos salvar
  int pointsToSave = preFlightQueueFull ? PRE_FLIGHT_QUEUE_SIZE : preFlightQueueIndex;
  
  // Salvar os pontos na ordem correta (mais antigo primeiro)
  for (int i = 0; i < pointsToSave; i++) {
    int queueIndex;
    if (preFlightQueueFull) {
      // Se a fila está cheia, começar do ponto mais antigo
      queueIndex = (preFlightQueueIndex + i) % PRE_FLIGHT_QUEUE_SIZE;
    } else {
      // Se a fila não está cheia, começar do índice 0
      queueIndex = i;
    }
    
    double altitude = preFlightQueue[queueIndex];
    saveAltitudeToEEPROM(altitude, recordCounter);
    recordCounter++;
    
    Serial.print(F("Ponto pré-voo #"));
    Serial.print(i);
    Serial.print(F(": "));
    Serial.print(altitude);
    Serial.println(F(" m"));
  }
  
  Serial.print(F("Total de pontos pré-voo salvos: "));
  Serial.println(pointsToSave);
}
