#include <SFE_BMP180.h>
#include <Wire.h>
#include <EEPROM.h>
//d2 buzzer d3 reset altimeter  d4 led vermelho d5 led verde d7 skib d12 ledligado
SFE_BMP180 pressure;
#define Skib 7
#define vermelho 4
#define Verde 5
#define buzzer 2
#define isWorking 12
#define ButtonAltimeter 3
#define MAX_ALTITUDE_ADDRESS 0
#define BASE_PRESSURE_ADDRESS 2
#define WINDOW_SIZE 5
#define ASCENT_THRESHOLD 2.0  // Altitude em metros para considerar que o foguete começou a subir

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
  Serial.println("Iniciando");
  pinMode(isWorking, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(Skib, OUTPUT);
  pinMode(Verde, OUTPUT);
  pinMode(vermelho, OUTPUT);
  pinMode(ButtonAltimeter, INPUT);
 

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

    if (alturasSuavizadas[0] <= maxAltitude - 5.0 && armado &&
        abs(alturasSuavizadas[0] - alturasSuavizadas[1]) < 5.0 &&
        abs(alturasSuavizadas[1] - alturasSuavizadas[2]) < 5.0) {
      ativaSkip();
      armado = false;
      isDescending = true;
      saveMaxAltitudeToEEPROM((short int)(maxAltitude * 10));
     
    }

    if (!armado) {
      contadorDeTempo++;
      if (contadorDeTempo >= 400) {
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
  double T, P;
  status = pressure.startTemperature();
  if (status != 0) {
    delay(status);
    status = pressure.getTemperature(T);
    if (status != 0) {
      status = pressure.startPressure(1);
      if (status != 0) {
        delay(status);
        status = pressure.getPressure(P, T);
        if (status != 0) {
          return P;
        }
      }
    }
  }
  return -1;
}

void ativaSkip() {
  digitalWrite(Skib, HIGH);
  ledVermelho(true);
}

void desativarSkib() {
  digitalWrite(Skib, LOW);
  ledVermelho(false);
}

void saveMaxAltitudeToEEPROM(short int maxAltitudeInt) {
  EEPROM.put(MAX_ALTITUDE_ADDRESS, maxAltitudeInt);
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
  digitalWrite(Verde, estado ? HIGH : LOW);
}

void ledVermelho(bool estado) {
  digitalWrite(vermelho, estado ? HIGH : LOW);
}

// Atualiza o LED "working" (pino 12) sem bloquear
void updateWorkingLED() {
  unsigned long currentMillis = millis();

  // Verifica onde estamos no ciclo de piscada
  if (currentMillis - previousBlinkMillis >= BLINK_INTERVAL) {
    // Inicia um novo ciclo
    previousBlinkMillis = currentMillis;
    ledWorkingState = HIGH;
    digitalWrite(isWorking, ledWorkingState);
  }
  // Se já passou o tempo de ficar ligado, desliga
  else if (currentMillis - previousBlinkMillis >= BLINK_ON_TIME) {
    ledWorkingState = LOW;
    digitalWrite(isWorking, ledWorkingState);
  }
}

// Dispara o buzzer em tom fixo sem travar o loop
void updateBuzzerPeriodic() {
  unsigned long now = millis();

  if (!isBeeping) {
    // hora de iniciar um novo beep?
    if (now - previousBeepTime >= BEEP_PERIOD) {
      tone(buzzer, BEEP_FREQUENCY);  // começa o tom
      isBeeping = true;
      previousBeepTime = now;
    }
  }
  else {
    // já tocou o suficiente?
    if (now - previousBeepTime >= BEEP_DURATION) {
      noTone(buzzer);               // para o tom
      isBeeping = false;
      // deixa previousBeepTime marcado para contar o próximo período
      previousBeepTime = now;
    }
  }
}
