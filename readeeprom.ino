// Arduino Nano Sketch: Visualizador de dados de voo da EEPROM
// Lê e interpreta a trajetória do foguete armazenada na EEPROM

#include <EEPROM.h>

// Constantes para identificar tipos de dados na EEPROM
#define MAX_ALTITUDE_ADDRESS 0        // Altitude máxima (2 bytes - short int)
#define BASE_PRESSURE_ADDRESS 2       // Pressão base (4 bytes - double)
#define FLIGHT_DATA_START_ADDRESS 6   // Início dos dados de voo
#define ALTITUDE_SCALE_FACTOR 10.0    // Fator de escala para converter altitude
#define RECORD_INTERVAL_ASCENT 50     // 50ms durante a subida
#define RECORD_INTERVAL_DESCENT 200   // 200ms durante a descida

// Flag para controle de modo de operação
bool useOriginalBaseline = true;   // Por padrão, usa a pressão base original da EEPROM
int skipPoints = 0;                // Número de pontos iniciais a ignorar

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // Aguardar conexão serial
  }

  Serial.println("Lendo dados de voo da EEPROM...");

  // Leitura da altitude máxima
  short int maxAltitudeInt;
  EEPROM.get(MAX_ALTITUDE_ADDRESS, maxAltitudeInt);
  float maxAltitude = maxAltitudeInt / ALTITUDE_SCALE_FACTOR;
  
  // Leitura da pressão base
  double basePressure;
  EEPROM.get(BASE_PRESSURE_ADDRESS, basePressure);
  
  Serial.println("\n------ DADOS IMPORTANTES ------");
  Serial.print("Altitude Máxima: ");
  Serial.print(maxAltitude, 1);
  Serial.println(" metros");
  
  Serial.print("Pressão Base: ");
  Serial.print(basePressure);
  Serial.println(" hPa");
  Serial.println("-----------------------------");
  
  // Menu com nova opção para pular pontos iniciais
  Serial.println("\nOPÇÕES:");
  Serial.println("1 - Mostrar trajetória completa");
  Serial.println("2 - Ignorar primeiros N pontos (para recuperar voo anterior)");
  Serial.println("d - Ver dump completo da EEPROM");
  
  while (!Serial.available()) {
    // Aguardar entrada
  }
  
  char choice = Serial.read();
  
  // Verificar opção escolhida
  if (choice == 'd' || choice == 'D') {
    dumpCompleteEEPROM();
    return;
  }
  else if (choice == '2') {
    // Ignorar pontos iniciais
    Serial.flush();
    Serial.println("\nDigite o número de pontos iniciais a ignorar:");
    
    // Aguardar entrada numérica
    skipPoints = 0;
    while (true) {
      if (Serial.available()) {
        char c = Serial.read();
        
        // Se pressionar Enter, terminou a entrada
        if (c == '\n' || c == '\r') {
          break;
        }
        // Se for dígito, adicionar ao número
        else if (c >= '0' && c <= '9') {
          skipPoints = skipPoints * 10 + (c - '0');
        }
      }
    }
    
    Serial.print("Ignorando os primeiros ");
    Serial.print(skipPoints);
    Serial.println(" pontos da trajetória...");
  }
  
  // Mostrar dados de voo em formato simplificado
  int altitudePeak = 0;
  int peakIndex = 0;
  int totalPoints = 0;
  
  // Primeiro loop para encontrar o total de pontos e o pico de altitude
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
        totalPoints = i;
        break;
      }
    }
    
    totalPoints = i + 1;
    
    // Encontrar o pico (considerar os pontos ignorados também)
    if (altitudeValue > altitudePeak) {
      altitudePeak = altitudeValue;
      peakIndex = i;
    }
  }
  
  // Verificar se temos pontos suficientes após ignorar os N primeiros
  if (skipPoints >= totalPoints) {
    Serial.println("\nERRO: Número de pontos a ignorar é maior que o total de pontos na trajetória!");
    Serial.print("Total de pontos na EEPROM: ");
    Serial.println(totalPoints);
    return;
  }
  
  // Mostrar trajetória simplificada
  Serial.println("\n------ TRAJETÓRIA DO VOO ------");
  Serial.println("Ponto\tTempo(ms)\tAltitude(m)\tFase");
  
  int validPoints = 0;
  for (int i = skipPoints; i < totalPoints; i++) {
    int addr = FLIGHT_DATA_START_ADDRESS + (i * 2);
    if (addr >= 512) break;
    
    short int altitudeValue;
    EEPROM.get(addr, altitudeValue);
    
    float altitudeMeters = altitudeValue / ALTITUDE_SCALE_FACTOR;
    validPoints++;
    
    // Fase (subida ou descida)
    String phase = (i >= peakIndex) ? "DESCIDA" : "SUBIDA";
    
    // Calcular o tempo estimado (relativo ao início da trajetória, após os pontos ignorados)
    float estimatedTime;
    if (i <= peakIndex) {
      estimatedTime = (i - skipPoints) * RECORD_INTERVAL_ASCENT;
    } else {
      estimatedTime = (peakIndex - skipPoints) * RECORD_INTERVAL_ASCENT + 
                     (i - peakIndex) * RECORD_INTERVAL_DESCENT;
      if (estimatedTime < 0) estimatedTime = 0; // Evitar tempos negativos
    }
    
    // Imprimir dados de forma mais limpa
    Serial.print(i - skipPoints);  // Ponto relativo (iniciando em 0 após os ignorados)
    Serial.print("\t");
    Serial.print(estimatedTime);
    Serial.print("\t\t");
    Serial.print(altitudeMeters, 1);
    Serial.print("\t\t");
    Serial.println(phase);
  }
  
  // Resumo dos dados de voo - usando a altitude máxima consistente
  Serial.println("\n------ RESUMO DO VOO ------");
  Serial.print("Altitude máxima: ");
  Serial.print(maxAltitude, 1);  // Usar o mesmo valor mostrado no início
  Serial.println(" metros");
  
  // Ajustar o cálculo do tempo baseado nos pontos ignorados
  int adjustedPeakIndex = peakIndex - skipPoints;
  if (adjustedPeakIndex < 0) {
    Serial.println("Aviso: O apogeu ocorreu nos pontos ignorados!");
    adjustedPeakIndex = 0;
  }
  
  Serial.print("Tempo até o apogeu: ");
  Serial.print((adjustedPeakIndex * RECORD_INTERVAL_ASCENT) / 1000.0, 1);
  Serial.println(" segundos");
  
  Serial.print("Duração do voo: ");
  float flightDuration;
  if (validPoints <= adjustedPeakIndex) {
    flightDuration = validPoints * RECORD_INTERVAL_ASCENT;
  } else {
    flightDuration = adjustedPeakIndex * RECORD_INTERVAL_ASCENT + 
                     (validPoints - adjustedPeakIndex) * RECORD_INTERVAL_DESCENT;
  }
  Serial.print(flightDuration / 1000.0, 1);
  Serial.println(" segundos");
  
  Serial.print("Total de pontos visualizados: ");
  Serial.print(validPoints);
  Serial.print(" (ignorados ");
  Serial.print(skipPoints);
  Serial.println(" pontos iniciais)");
  
  // Aviso sobre possíveis inconsistências
  if (skipPoints > 0) {
    Serial.println("\nNOTA: A trajetória mostrada pode ter inconsistências devido aos pontos ignorados.");
  }
}

void loop() {
  // Nada para fazer aqui
}

void dumpCompleteEEPROM() {
  Serial.println("\n------ DUMP COMPLETO DA EEPROM ------");
  Serial.println("Endereço\tValor\tInterpretação");
  
  // Mostrar altitude máxima
  short int maxAltitude;
  EEPROM.get(MAX_ALTITUDE_ADDRESS, maxAltitude);
  Serial.print("0-1\t\t");
  Serial.print(maxAltitude);
  Serial.print("\tAltitude máxima: ");
  Serial.print(maxAltitude / ALTITUDE_SCALE_FACTOR, 1);
  Serial.println(" metros");
  
  // Mostrar pressão base
  double baseP;
  EEPROM.get(BASE_PRESSURE_ADDRESS, baseP);
  Serial.print("2-5\t\t");
  Serial.print(baseP);
  Serial.println("\tPressão base (hPa)");
  
  // Mostrar dados de voo
  Serial.println("\n--- PONTOS DE ALTITUDE ---");
  Serial.println("Endereço\tPonto\tValor\tAltitude(m)");
  
  for (int i = 0; i < 250; i++) {
    int addr = FLIGHT_DATA_START_ADDRESS + (i * 2);
    if (addr >= 512) break;
    
    short int altitudeValue;
    EEPROM.get(addr, altitudeValue);
    
    // Parar se encontrar sequência de zeros
    if (altitudeValue == 0 && i > 5) {
      short int prevValue;
      EEPROM.get(addr - 2, prevValue);
      if (prevValue == 0 || abs(prevValue) < 10) {
        break;
      }
    }
    
    // Mostrar dados de altitude em formato mais limpo
    Serial.print(addr);
    Serial.print("-");
    Serial.print(addr+1);
    Serial.print("\t\t");
    Serial.print(i);
    Serial.print("\t");
    Serial.print(altitudeValue);
    Serial.print("\t");
    Serial.println(altitudeValue / ALTITUDE_SCALE_FACTOR, 1);
    
    // Pequeno atraso para legibilidade
    delay(1);
  }
}
                        
