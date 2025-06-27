// Arduino Nano Sketch: Read EEPROM Addresses 0 to 512 and Print with Address Labels
// 
// This sketch reads the EEPROM contents from address 0 to 512
// and prints each byte in the Serial Monitor with its address.
// Ensure you have <EEPROM.h> available (built-in) and a Serial Monitor open.

#include <EEPROM.h>

// Constantes para identificar tipos de dados especiais na EEPROM
#define MAX_ALTITUDE_ADDRESS 0        // Endereço da altitude máxima (short int - 2 bytes)
#define BASE_PRESSURE_ADDRESS 2       // Endereço da pressão base (double - 4 bytes) 
#define FLIGHT_DATA_START_ADDRESS 6   // Início dos dados do voo (short int - 2 bytes por ponto)
#define MAX_FLIGHT_DATA_POINTS 497    // Total de pontos (EEPROM: 1000 - 6 = 994 bytes / 2 = 497)
#define ALTITUDE_SCALE_FACTOR 10.0    // Fator de escala usado para converter altitude em short int
#define PRE_FLIGHT_QUEUE_SIZE 4       // Tamanho da fila de pontos pré-voo
#define RECORD_INTERVAL_ASCENT 50     // 50ms durante a subida (maior frequência)
#define RECORD_INTERVAL_DESCENT 200   // 200ms durante a descida (menor frequência)
#define EEPROM_SIZE 1000              // Tamanho total da EEPROM disponível

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
  float maxAltitude = maxAltitudeInt / ALTITUDE_SCALE_FACTOR;
  
  // Leitura da pressão base (endereços 2-5)
  double basePressure;
  EEPROM.get(BASE_PRESSURE_ADDRESS, basePressure);
  
  Serial.println("=====================================");
  Serial.println("        DADOS DO VOO - EEPROM       ");
  Serial.println("=====================================");
  Serial.print("Altitude Máxima: ");
  Serial.print(maxAltitude, 1);
  Serial.print(" m (raw: ");
  Serial.print(maxAltitudeInt);
  Serial.println(")");
  Serial.print("Pressão Base: ");
  Serial.print(basePressure, 2);
  Serial.println(" hPa");
  Serial.println("-------------------------------------");

  Serial.println("-------------------------------------");

  // Análise dos dados de voo para encontrar o fim válido
  int validDataPoints = 0;
  int altitudePeak = 0;
  int peakIndex = 0;
  
  // Primeiro: determinar quantos pontos são válidos
  for (int i = 0; i < MAX_FLIGHT_DATA_POINTS; i++) {
    int addr = FLIGHT_DATA_START_ADDRESS + (i * 2);
    if (addr >= EEPROM_SIZE) break;
    
    short int altitudeValue;
    EEPROM.get(addr, altitudeValue);
    
    // Critério melhorado para detectar fim dos dados válidos
    if (i > PRE_FLIGHT_QUEUE_SIZE) {
      // Após os pontos pré-voo, verificar padrões de dados inválidos
      if (altitudeValue == 0 || altitudeValue == -1 || 
          abs(altitudeValue) > 32000 || // Valores muito extremos
          (i > 10 && abs(altitudeValue) < 5)) { // Muito próximo de zero após início do voo
        
        // Verificar se realmente é o fim (próximos valores também inválidos)
        bool isEnd = true;
        for (int j = 1; j <= 3 && (i + j) < MAX_FLIGHT_DATA_POINTS; j++) {
          short int nextValue;
          EEPROM.get(FLIGHT_DATA_START_ADDRESS + ((i + j) * 2), nextValue);
          if (nextValue != 0 && nextValue != -1 && abs(nextValue) < 32000) {
            isEnd = false;
            break;
          }
        }
        if (isEnd) break;
      }
    }
    
    validDataPoints++;
    
    // Encontrar o pico (ignorar pontos pré-voo)
    if (i >= PRE_FLIGHT_QUEUE_SIZE && altitudeValue > altitudePeak) {
      altitudePeak = altitudeValue;
      peakIndex = i;
    }
  }

  Serial.print("Pontos válidos encontrados: ");
  Serial.println(validDataPoints);
  
  // Opção para dump completo
  Serial.println("Opções:");
  Serial.println("  'd' = Dump completo byte-a-byte");
  Serial.println("  'r' = Resumo rápido do voo");
  Serial.println("  Qualquer outra = Dados completos de voo");
  
  while (!Serial.available()) {
    // Aguardar entrada
  }
  
  char option = Serial.read();
  Serial.println();
  
  if (option == 'd') {
    dumpCompleteEEPROM();
    return;
  } else if (option == 'r') {
    showFlightSummary(validDataPoints, altitudePeak, peakIndex);
    return;
  }
  }

  // Mostrar dados de voo em formato melhorado
  Serial.println("=====================================");
  Serial.println("         DADOS DE TRAJETÓRIA         ");
  Serial.println("=====================================");
  Serial.println("Idx\tTempo(s)\tAlt(m)\tTipo\t\tFase");
  Serial.println("-------------------------------------");
  
  // Exibir apenas dados válidos
  for (int i = 0; i < validDataPoints; i++) {
    int addr = FLIGHT_DATA_START_ADDRESS + (i * 2);
    
    short int altitudeValue;
    EEPROM.get(addr, altitudeValue);
    
    float altitudeMeters = altitudeValue / ALTITUDE_SCALE_FACTOR;
    
    // Determinar tipo e fase
    String dataType, phase;
    float estimatedTime;
    
    if (i < PRE_FLIGHT_QUEUE_SIZE) {
      dataType = "PRE-VOO";
      phase = "PREPARAÇÃO";
      estimatedTime = (i - PRE_FLIGHT_QUEUE_SIZE) * RECORD_INTERVAL_ASCENT / 1000.0;
    } else {
      dataType = "VÔOO";
      if (i < peakIndex) {
        phase = "SUBIDA";
        estimatedTime = (i - PRE_FLIGHT_QUEUE_SIZE) * RECORD_INTERVAL_ASCENT / 1000.0;
      } else {
        phase = "DESCIDA";
        estimatedTime = ((peakIndex - PRE_FLIGHT_QUEUE_SIZE) * RECORD_INTERVAL_ASCENT + 
                        (i - peakIndex) * RECORD_INTERVAL_DESCENT) / 1000.0;
      }
    }
    
    // Formatação melhorada
    Serial.print(i);
    Serial.print("\t");
    Serial.print(estimatedTime, 2);
    Serial.print("\t\t");
    Serial.print(altitudeMeters, 1);
    Serial.print("\t");
    Serial.print(dataType);
    Serial.print("\t");
    Serial.println(phase);
    
    // Pausa a cada 20 linhas para não sobrecarregar
    if ((i + 1) % 20 == 0 && i < validDataPoints - 1) {
      Serial.println("--- Pressione qualquer tecla para continuar ---");
      while (!Serial.available()) { }
      Serial.read();
    }
  }
  
  showFlightSummary(validDataPoints, altitudePeak, peakIndex);
}

void showFlightSummary(int validPoints, int altitudePeak, int peakIndex) {
  Serial.println("\n=====================================");
  Serial.println("           RESUMO DO VÔO            ");
  Serial.println("=====================================");
  
  float maxAltitudeMeters = altitudePeak / ALTITUDE_SCALE_FACTOR;
  Serial.print("🚀 Altitude máxima: ");
  Serial.print(maxAltitudeMeters, 1);
  Serial.println(" metros");
  
  Serial.print("📊 Total de amostras: ");
  Serial.println(validPoints);
  
  Serial.print("⏱️  Apogeu na amostra: #");
  Serial.println(peakIndex);
  
  float timeToApogee = (peakIndex - PRE_FLIGHT_QUEUE_SIZE) * RECORD_INTERVAL_ASCENT / 1000.0;
  Serial.print("⏰ Tempo até apogeu: ");
  Serial.print(timeToApogee, 2);
  Serial.println(" segundos");
  
  // Duração total estimada
  float totalDuration;
  if (validPoints <= peakIndex) {
    totalDuration = validPoints * RECORD_INTERVAL_ASCENT / 1000.0;
  } else {
    totalDuration = ((peakIndex - PRE_FLIGHT_QUEUE_SIZE) * RECORD_INTERVAL_ASCENT + 
                    (validPoints - peakIndex) * RECORD_INTERVAL_DESCENT) / 1000.0;
  }
  
  Serial.print("🕐 Duração total: ");
  Serial.print(totalDuration, 2);
  Serial.println(" segundos");
  
  // Estatísticas adicionais
  int preFlightPoints = min(PRE_FLIGHT_QUEUE_SIZE, validPoints);
  int ascentPoints = max(0, min(peakIndex - PRE_FLIGHT_QUEUE_SIZE, validPoints - PRE_FLIGHT_QUEUE_SIZE));
  int descentPoints = max(0, validPoints - peakIndex);
  
  Serial.print("📈 Pontos pré-voo: ");
  Serial.println(preFlightPoints);
  Serial.print("📈 Pontos de subida: ");
  Serial.println(ascentPoints);
  Serial.print("📉 Pontos de descida: ");
  Serial.println(descentPoints);
  
  Serial.println("=====================================");
}

void loop() {
  // Nothing to do here
}

void dumpCompleteEEPROM() {
  Serial.println("=====================================");
  Serial.println("      DUMP COMPLETO DA EEPROM       ");
  Serial.println("=====================================");
  Serial.println("Endereço\tValor\tInterpretação");
  Serial.println("-------------------------------------");
  
  for (int addr = 0; addr < min(512, EEPROM_SIZE); addr++) {
    byte value = EEPROM.read(addr);

    Serial.print(addr);
    Serial.print("\t\t");
    Serial.print(value);
    Serial.print("\t");

    // Interpretações melhoradas
    if (addr == 0 || addr == 1) {
      if (addr == 0) {
        short int maxAlt;
        EEPROM.get(0, maxAlt);
        Serial.print("Alt. máxima: ");
        Serial.print(maxAlt / ALTITUDE_SCALE_FACTOR, 1);
        Serial.print("m");
      } else {
        Serial.print("(parte alt. máxima)");
      }
    } 
    else if (addr >= 2 && addr <= 5) {
      if (addr == 2) {
        double pressure;
        EEPROM.get(2, pressure);
        Serial.print("Pressão base: ");
        Serial.print(pressure, 2);
        Serial.print(" hPa");
      } else {
        Serial.print("(parte pressão)");
      }
    }
    else if (addr >= FLIGHT_DATA_START_ADDRESS && addr < FLIGHT_DATA_START_ADDRESS + (MAX_FLIGHT_DATA_POINTS * 2)) {
      if ((addr - FLIGHT_DATA_START_ADDRESS) % 2 == 0) {
        short int altValue;
        EEPROM.get(addr, altValue);
        int sampleNum = (addr - FLIGHT_DATA_START_ADDRESS) / 2;
        
        Serial.print("Amostra #");
        Serial.print(sampleNum);
        Serial.print(": ");
        Serial.print(altValue / ALTITUDE_SCALE_FACTOR, 1);
        Serial.print("m");
      } else {
        Serial.print("(parte amostra)");
      }
    } else {
      Serial.print("(não usado)");
    }
    
    Serial.println();

    // Pausa a cada 30 linhas
    if ((addr + 1) % 30 == 0 && addr < 511) {
      Serial.println("--- Pressione qualquer tecla para continuar ---");
      while (!Serial.available()) { }
      Serial.read();
    }
  }
  
  Serial.println("=====================================");
}
                        
