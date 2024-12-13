#include <SFE_BMP180.h>
#include <Wire.h>
#include <EEPROM.h>

SFE_BMP180 pressure;
#define Skib 8
#define vermelho 4
#define Verde 2
#define ButtonAltimeter 3
#define ButtonIsOperating 5
#define MAX_ALTITUDE_ADDRESS 0
#define BASE_PRESSURE_ADDRESS 2
#define WINDOW_SIZE 5 

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

double alturasSuavizadas[3] = {0, 0, 0}; 
double altitudeReadings[WINDOW_SIZE] = {0}; 
int currentReadingIndex = 0;
int totalReadings = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando");

  pinMode(Skib, OUTPUT);
  pinMode(Verde, OUTPUT);
  pinMode(vermelho, OUTPUT);
  pinMode(ButtonAltimeter, INPUT);
  pinMode(ButtonIsOperating, INPUT)

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
  if (digitalRead(ButtonAltimeter) == HIGH) {
    resetBaseline();
    for (int i = 0; i < 10; i++) {
      ledVerde(false);
      ledVermelho(true);
      delay(200);
      ledVerde(true);
      ledVermelho(false);
      delay(200);
    }
  }

  double P = getPressure();
  if (P != -1) {
    double rawAltitude = pressure.altitude(P, baseline);
    double smoothedAltitude = getSmoothedAltitude(rawAltitude);

    alturasSuavizadas[2] = alturasSuavizadas[1];
    alturasSuavizadas[1] = alturasSuavizadas[0];
    alturasSuavizadas[0] = smoothedAltitude;

    Serial.println(smoothedAltitude);

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

void resetBaseline() {
  baseline = getPressure();
  saveBaselineToEEPROM(baseline);
  Serial.println("Baseline resetado.");
}

void ledVerde(bool estado) {
  digitalWrite(Verde, estado ? HIGH : LOW);
}

void ledVermelho(bool estado) {
  digitalWrite(vermelho, estado ? HIGH : LOW);
}
