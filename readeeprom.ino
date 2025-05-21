// Arduino Nano Sketch: Read EEPROM Addresses 0 to 512 and Print with Address Labels
// 
// This sketch reads the EEPROM contents from address 0 to 512
// and prints each byte in the Serial Monitor with its address.
// Ensure you have <EEPROM.h> available (built-in) and a Serial Monitor open.

#include <EEPROM.h>

// Constantes para identificar tipos de dados especiais na EEPROM
#define MAX_ALTITUDE_ADDRESS 0    // Endereço da altitude máxima (short int - 2 bytes)
#define BASE_PRESSURE_ADDRESS 2   // Endereço da pressão base (double - 4 bytes) 
#define FLIGHT_DATA_START_ADDRESS 6  // Início dos dados do voo (short int - 2 bytes por ponto)
#define ALTITUDE_SCALE_FACTOR 10.0    // Fator de escala usado para converter altitude em short int
#define RECORD_INTERVAL_ASCENT 50     // 50ms durante a subida (maior frequência)
#define RECORD_INTERVAL_DESCENT 200   // 200ms durante a descida (menor frequência)

void setup() {
  // Initialize serial communication at 9600 baud
  Serial.begin(9600);
  // Wait for serial port to connect (useful for Leonardo/Micro)
  while (!Serial) {
    ;
  }

  Serial.println("Starting EEPROM dump...");

  // Leitura e interpretação da altitude máxima (endereços 0-1)
  short int maxAltitudeInt;
  EEPROM.get(MAX_ALTITUDE_ADDRESS, maxAltitudeInt);
  float maxAltitude = maxAltitudeInt / 10.0;
  
  Serial.println("------ DADOS IMPORTANTES ------");
  Serial.print("Altitude Máxima: ");
  Serial.print(maxAltitude);
  Serial.println(" metros");
  
  // Leitura da pressão base (endereços 2-5)
  double basePressure;
  EEPROM.get(BASE_PRESSURE_ADDRESS, basePressure);
  Serial.print("Pressão Base: ");
  Serial.print(basePressure);
  Serial.println(" hPa");
  Serial.println("-----------------------------");
  Serial.println();

  // Read and print each EEPROM address from 0 to 512
  Serial.println("------ DUMP COMPLETO DA EEPROM ------");
  for (int addr = 0; addr < 512; addr++) {
    // Read a single byte from EEPROM
    byte value = EEPROM.read(addr);

    // Print address label and its stored value
    Serial.print("Address ");
    Serial.print(addr);
    Serial.print(": ");
    Serial.print(value);

    // Adicionar interpretações especiais para cada tipo de dado
    if (addr == 0 || addr == 1) {
      // Parte da altitude máxima (short int)
      if (addr == 0) {
        Serial.print(" (Parte da altitude máxima)");
      }
    } 
    else if (addr >= 2 && addr <= 5) {
      // Parte da pressão base (double)
      Serial.print(" (Parte da pressão base)");
    }
    else if (addr >= FLIGHT_DATA_START_ADDRESS) {
      // Verificar se é o primeiro byte de um short int (endereço par)
      if ((addr - FLIGHT_DATA_START_ADDRESS) % 2 == 0) {
        // Ler o short int completo
        short int altitudeValue;
        EEPROM.get(addr, altitudeValue);
        
        // Calcular o número da amostra
        int sampleNumber = (addr - FLIGHT_DATA_START_ADDRESS) / 2;
        
        Serial.print(" (Altitude #");
        Serial.print(sampleNumber);
        Serial.print(": ");
        Serial.print(altitudeValue / ALTITUDE_SCALE_FACTOR, 1);
        Serial.print(" metros)");
      } else {
        Serial.print(" (Parte de short int)");
      }
    }
    
    Serial.println();

    // Small delay for readability (optional)
    delay(5);
  }

  // Mostrar dados de voo em formato tabular mais legível e verificar se há evidência de apogeu
  Serial.println();
  Serial.println("------ DADOS DE VOO ------");
  Serial.println("Amostra\tTempo(ms)\tAltitude(m)\tEstimativa Fase");
  
  int altitudePeak = 0;
  int peakIndex = 0;
  boolean apogeeDetected = false;
  float prevAltitude = -999;
  int estimatedApogeeIndex = -1;
  
  // Primeiro loop para encontrar o pico de altitude
  for (int i = 0; i < 250; i++) {
    int addr = FLIGHT_DATA_START_ADDRESS + (i * 2);
    if (addr >= 512) break; 
    
    short int altitudeValue;
    EEPROM.get(addr, altitudeValue);
    
    if (altitudeValue == 0 && i > 5) {
      // Verificar se é o fim dos dados válidos
      short int prevValue;
      EEPROM.get(addr - 2, prevValue);
      if (prevValue == 0 || abs(prevValue) < 10) {
        break;
      }
    }
    
    // Encontrar o pico
    if (altitudeValue > altitudePeak) {
      altitudePeak = altitudeValue;
      peakIndex = i;
    }
  }
  
  // Segundo loop para exibir dados com informações de fase
  float timeAdjustment = 0;
  for (int i = 0; i < 250; i++) {
    int addr = FLIGHT_DATA_START_ADDRESS + (i * 2);
    if (addr >= 512) break;
    
    short int altitudeValue;
    EEPROM.get(addr, altitudeValue);
    
    if (altitudeValue == 0 && i > 5) {
      // Verificar se é o fim dos dados válidos
      short int prevValue;
      EEPROM.get(addr - 2, prevValue);
      if (prevValue == 0 || abs(prevValue) < 10) {
        break;
      }
    }
    
    float altitudeMeters = altitudeValue / ALTITUDE_SCALE_FACTOR;
    
    // Estimar a fase (subida ou descida) com base na posição relativa ao pico
    String phase = "SUBIDA";
    int timeInterval = RECORD_INTERVAL_ASCENT;
    
    if (i >= peakIndex) {
      phase = "DESCIDA";
      timeInterval = RECORD_INTERVAL_DESCENT;
    }
    
    // Calcular o tempo estimado considerando intervalos diferentes para subida e descida
    float estimatedTime;
    if (i <= peakIndex) {
      estimatedTime = i * RECORD_INTERVAL_ASCENT;  // Tempo durante a subida
    } else {
      // Tempo até o pico (usando intervalo de subida) + tempo adicional (usando intervalo de descida)
      estimatedTime = peakIndex * RECORD_INTERVAL_ASCENT + 
                     (i - peakIndex) * RECORD_INTERVAL_DESCENT;
    }
    
    Serial.print(i);
    Serial.print("\t");
    Serial.print(estimatedTime);
    Serial.print("\t\t");
    Serial.print(altitudeMeters, 1);
    Serial.print("\t\t");
    Serial.println(phase);
  }
  
  Serial.println("\n------ RESUMO ------");
  Serial.print("Altitude máxima: ");
  Serial.print(altitudePeak / ALTITUDE_SCALE_FACTOR, 1);
  Serial.println(" metros");
  
  Serial.print("Amostra do apogeu: #");
  Serial.println(peakIndex);
  
  Serial.print("Tempo estimado até o apogeu: ");
  Serial.print(peakIndex * RECORD_INTERVAL_ASCENT);
  Serial.println(" ms");
  
  Serial.println("EEPROM dump complete.");
}

void loop() {
  // Nothing to do here
}
