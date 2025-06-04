# Análise Detalhada das Funções - RecHugo

## FUNÇÕES PRINCIPAIS - ANÁLISE DETALHADA

### 1. setup()
```cpp
void setup() {
  // INICIALIZAÇÃO BÁSICA
  Serial.begin(9600);                    // Comunicação serial para debug
  pinMode(PIN_LED_WORKING, OUTPUT);      // LED indicador sistema funcionando
  pinMode(PIN_BUZZER, OUTPUT);           // Buzzer para feedback sonoro
  pinMode(PIN_SKIB, OUTPUT);             // Controle do sistema de recuperação
  pinMode(PIN_LED_GREEN, OUTPUT);        // LED status sensor (verde = OK)
  pinMode(PIN_LED_RED, OUTPUT);          // LED status SKIB (vermelho = ativo)
  pinMode(PIN_BUTTON_ALTIMETER, INPUT);  // Botão reset (não implementado)

  // INICIALIZAÇÃO DO SENSOR
  sensorConnected = pressure.begin();    // Tenta conectar BMP180
  if (!sensorConnected) {
    // Sensor falhou - indica erro
    ledVermelho(true);
    ledVerde(false);
  } else {
    // Sensor OK - carrega configurações
    baseline = readBaselineFromEEPROM();
    ledVerde(true);
    ledVermelho(false);
  }
  
  resetBaseline();                       // Calibra pressão atual
  initializePreFlightQueue();            // Inicializa fila pré-voo
}
```

**Função:** Configuração inicial do sistema
**Tempo de execução:** ~1-2 segundos
**Dependências:** Sensor BMP180, EEPROM

---

### 2. loop() - Loop Principal
```cpp
void loop() {
  double P = getPressure();              // Lê pressão do sensor
  if (P != -1) {                        // Verifica se leitura é válida
    // PROCESSAMENTO DE DADOS
    double rawAltitude = pressure.altitude(P, baseline);
    smoothedAltitude = getSmoothedAltitude(rawAltitude);
    updateSmoothAltitudeHistory();
    
    // MÁQUINA DE ESTADOS
    switch (currentState) {
      case PRE_FLIGHT:  handlePreFlightState();  break;
      case ASCENT:      handleAscentState();     break;
      case RECOVERY:    handleRecoveryState();   break;
    }
    
    // VERIFICAÇÕES E ATUALIZAÇÕES
    checkStateTransitions();             // Verifica mudanças de estado
    if (smoothedAltitude > maxAltitude) {
      maxAltitude = smoothedAltitude;    // Atualiza altitude máxima
    }
    
    // GERENCIAMENTO SKIB
    if (!armado && !skibDeactivated && 
        millis() - skibActivatedAt >= SKIB_DEACTIVATION_TIME) {
      desativarSkib();
      skibDeactivated = true;
    }
  }
  
  // ATUALIZAÇÕES PERIÓDICAS
  updateSkibBuzzer();      // Gerencia buzzer do SKIB
  atualizarEstadoSensor(); // Verifica conexão do sensor
  updateWorkingLED();      // Atualiza LED de funcionamento
  updateBuzzerByState();   // Buzzer baseado no estado
}
```

**Função:** Loop principal, executado continuamente
**Frequência:** ~20 Hz (limitado pelo sensor)
**Crítico:** Não deve ter delays bloqueantes

---

### 3. getPressure() - Leitura do Sensor
```cpp
double getPressure() {
  char status;
  double T, P;
  
  // ETAPA 1: Medição de temperatura
  status = pressure.startTemperature();  // Inicia medição
  if (status == 0) return -1;            // Falha na inicialização
  delay(status);                         // Aguarda medição (~5ms)
  
  status = pressure.getTemperature(T);   // Lê temperatura
  if (status == 0) return -1;            // Falha na leitura
  
  // ETAPA 2: Medição de pressão
  status = pressure.startPressure(1);    // Inicia medição (resolução 1)
  if (status == 0) return -1;            // Falha na inicialização
  delay(status);                         // Aguarda medição (~8ms)
  
  status = pressure.getPressure(P, T);   // Lê pressão
  if (status == 0) return -1;            // Falha na leitura
  
  return P;                              // Retorna pressão válida
}
```

**Função:** Lê pressão barométrica do BMP180
**Tempo:** ~15ms (inclui delays necessários)
**Retorno:** Pressão em hPa ou -1 (erro)
**Nota:** Única função com delay - necessário para o sensor

---

### 4. getSmoothedAltitude() - Suavização
```cpp
double getSmoothedAltitude(double newAltitude) {
  // ADICIONA NOVA LEITURA À FILA CIRCULAR
  altitudeReadings[currentReadingIndex] = newAltitude;
  currentReadingIndex = (currentReadingIndex + 1) % WINDOW_SIZE;
  
  // CALCULA MÉDIA MÓVEL
  double sum = 0;
  int count = totalReadings < WINDOW_SIZE ? totalReadings : WINDOW_SIZE;
  for (int i = 0; i < count; i++) {
    sum += altitudeReadings[i];
  }
  
  totalReadings++;
  return sum / count;                    // Retorna média das últimas leituras
}
```

**Função:** Remove ruído das leituras de altitude
**Método:** Média móvel das últimas 5 leituras
**Vantagem:** Reduz oscilações, mantém responsividade
**Desvantagem:** Delay de ~0.25s para estabilizar

---

### 5. saveAltitudeToEEPROM() - Gravação de Dados
```cpp
void saveAltitudeToEEPROM(double altitude, unsigned int index) {
  short int altitudeScaled;
  
  // COMPRESSÃO COM VERIFICAÇÃO DE LIMITES
  if (altitude < -3276.8) {
    altitudeScaled = -32768;             // Limite inferior
  } else if (altitude > 3276.7) {
    altitudeScaled = 32767;              // Limite superior
  } else {
    altitudeScaled = (short int)(altitude * ALTITUDE_SCALE_FACTOR);
  }
  
  // CÁLCULO DO ENDEREÇO E GRAVAÇÃO
  int address = FLIGHT_DATA_START_ADDRESS + (index * 2);
  EEPROM.put(address, altitudeScaled);   // Grava 2 bytes
}
```

**Função:** Comprime e salva altitude na EEPROM
**Compressão:** double (8 bytes) → short int (2 bytes)
**Precisão:** 0.1 metros
**Capacidade:** 497 pontos de dados

---

### 6. Sistema de Estados - Funções de Gerenciamento

#### handlePreFlightState()
```cpp
void handlePreFlightState() {
  addToPreFlightQueue(smoothedAltitude); // Adiciona à fila circular
  
  // LOG DE DEBUG (não grava EEPROM)
  Serial.print("Pré-voo - Altitude: ");
  Serial.print(smoothedAltitude);
  Serial.println(" m (não gravando na EEPROM)");
}
```
**Função:** Monitora altitude, armazena em fila circular
**Gravação EEPROM:** NÃO (economiza espaço)
**Fila:** Últimas 4 altitudes para uso posterior

#### handleAscentState()
```cpp
void handleAscentState() {
  // GRAVAÇÃO COM ALTA FREQUÊNCIA
  if (millis() - lastRecordTime >= RECORD_INTERVAL_ASCENT && 
      recordCounter < MAX_FLIGHT_DATA_POINTS) {
    
    saveAltitudeToEEPROM(smoothedAltitude, recordCounter);
    recordCounter++;
    lastRecordTime = millis();
    
    // LOG DETALHADO
    Serial.print("ASCENSÃO - Ponto: ");
    Serial.print(recordCounter);
    Serial.print(", Tempo: ");
    Serial.print(lastRecordTime);
    Serial.print(" ms, Altura: ");
    Serial.print(smoothedAltitude);
    Serial.println(" m");
  }
}
```
**Função:** Grava dados de ascensão com alta resolução
**Frequência:** 50ms (20 Hz)
**Crítico:** Captura fase mais importante do voo

#### handleRecoveryState()
```cpp
void handleRecoveryState() {
  // GRAVAÇÃO COM BAIXA FREQUÊNCIA
  if (millis() - lastRecordTime >= RECORD_INTERVAL_DESCENT && 
      recordCounter < MAX_FLIGHT_DATA_POINTS) {
    
    saveAltitudeToEEPROM(smoothedAltitude, recordCounter);
    recordCounter++;
    lastRecordTime = millis();
    
    // LOG DE RECUPERAÇÃO
    Serial.print("RECUPERAÇÃO - Ponto: ");
    Serial.print(recordCounter);
    Serial.print(", Tempo: ");
    Serial.print(lastRecordTime);
    Serial.print(" ms, Altura: ");
    Serial.print(smoothedAltitude);
    Serial.println(" m");
  }
}
```
**Função:** Grava dados de descida conservando espaço
**Frequência:** 200ms (5 Hz)
**Objetivo:** Documenta recuperação, economiza EEPROM

---

### 7. checkStateTransitions() - Transições de Estado
```cpp
void checkStateTransitions() {
  switch (currentState) {
    case PRE_FLIGHT:
      // DETECÇÃO DE INÍCIO DE VOO
      if (smoothedAltitude >= ASCENT_THRESHOLD) {
        currentState = ASCENT;
        Serial.println("TRANSIÇÃO: PRE_FLIGHT -> ASCENT");
        savePreFlightPointsToEEPROM();   // Salva pontos anteriores
        lastRecordTime = millis();       // Inicializa timer
      }
      break;
      
    case ASCENT:
      // DETECÇÃO DE APOGEU
      if (alturasSuavizadas[0] <= maxAltitude - DESCENT_DETECTION_THRESHOLD && 
          armado) {
        Serial.println("TRANSIÇÃO: ASCENT -> RECOVERY");
        activateRecoverySystem();        // Ativa SKIB
        apogeuDetectado = true;
        isDescending = true;
      }
      break;
      
    case RECOVERY:
      // ESTADO FINAL - SEM TRANSIÇÕES
      break;
  }
}
```

**Função:** Gerencia mudanças automáticas de estado
**Triggers:**
- PRE_FLIGHT → ASCENT: altitude > 5m
- ASCENT → RECOVERY: descida de 5m do pico
**Crítico:** Determina momento de ativação do SKIB

---

### 8. activateRecoverySystem() - Ativação do SKIB
```cpp
void activateRecoverySystem() {
  ativaSkib();                          // Liga pino do SKIB
  armado = false;                       // Desarma sistema
  isDescending = true;                  // Flag de descida
  currentState = RECOVERY;              // Muda estado
  ledVermelho(HIGH);                    // Indica ativação
  skibActivatedAt = millis();           // Marca tempo
  skibDeactivated = false;              // Controle de timeout
  
  // SALVA DADOS CRÍTICOS
  saveMaxAltitudeToEEPROM((short int)(maxAltitude * 10));
  
  // BUZZER DE EMERGÊNCIA
  tone(PIN_BUZZER, BEEP_FREQUENCY_RECOVERY);
  skibBuzzerActive = true;
  skibBuzzerEndTime = millis() + SKIB_BUZZER_DURATION;
  
  Serial.println("Sistema de recuperação ativado com buzzer!");
}
```

**Função:** Ativa sistema de recuperação (paraquedas)
**Ações simultâneas:**
- Liga SKIB fisicamente
- Salva altitude máxima
- Buzzer de emergência (4s)
- Muda estado e flags
**Crítico:** Função mais importante - recuperação do foguete

---

### 9. Funções de Suporte

#### updateBuzzerByState()
```cpp
void updateBuzzerByState() {
  if (skibBuzzerActive) return;         // Prioridade ao buzzer SKIB
  
  unsigned long beepPeriod;
  unsigned int beepFrequency;
  
  // DEFINE PARÂMETROS POR ESTADO
  switch (currentState) {
    case PRE_FLIGHT:
      beepPeriod = BEEP_PERIOD_PREFLIGHT;      // 3000ms
      beepFrequency = BEEP_FREQUENCY_PREFLIGHT; // 1500Hz
      break;
    case ASCENT:
      beepPeriod = BEEP_PERIOD_ASCENT;         // 1000ms
      beepFrequency = BEEP_FREQUENCY_ASCENT;   // 2093Hz
      break;
    case RECOVERY:
      beepPeriod = BEEP_PERIOD_RECOVERY;       // 500ms
      beepFrequency = armado ? BEEP_FREQUENCY_RECOVERY : BEEP_FREQUENCY_SKIB;
      break;
  }
  
  // GERENCIA CICLO DO BUZZER
  if (!isBeeping && (millis() - previousBeepTime >= beepPeriod)) {
    tone(PIN_BUZZER, beepFrequency);
    isBeeping = true;
    previousBeepTime = millis();
  } else if (isBeeping && (millis() - previousBeepTime >= BEEP_DURATION)) {
    noTone(PIN_BUZZER);
    isBeeping = false;
    previousBeepTime = millis();
  }
}
```

**Função:** Feedback sonoro diferenciado por estado
**Objetivo:** Permite identificar fase do voo pelo som
**Estados audíveis:**
- Grave/lento: Preparação
- Médio/rápido: Voo ativo  
- Agudo/muito rápido: Emergência

---

## RESUMO DE PERFORMANCE

| Função | Tempo Execução | Frequência | Crítica |
|--------|----------------|------------|---------|
| getPressure() | ~15ms | ~20 Hz | Alta |
| saveAltitudeToEEPROM() | <1ms | Variável | Média |
| getSmoothedAltitude() | <1ms | ~20 Hz | Alta |
| activateRecoverySystem() | ~5ms | 1x | Crítica |
| updateBuzzerByState() | <1ms | ~1000 Hz | Baixa |
| checkStateTransitions() | <1ms | ~20 Hz | Alta |

**Total do loop:** ~20ms → Frequência máxima ~50 Hz
**Limitação:** Sensor BMP180 (delays necessários)
