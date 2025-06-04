# Documentação Técnica - Sistema de Altímetro RecHugo

## 1. VISÃO GERAL

O RecHugo.ino é um sistema de altímetro para foguetes modelismo que utiliza sensor barométrico BMP180 para:
- Monitorar altitude em tempo real
- Detectar fases do voo automaticamente
- Acionar sistema de recuperação (SKIB)
- Gravar dados de voo na EEPROM
- Fornecer feedback sonoro e visual

## 2. HARDWARE NECESSÁRIO

### Componentes:
- **Arduino Nano** (ou compatível)
- **Sensor BMP180** (pressão/temperatura)
- **Buzzer** (pino D2)
- **LEDs indicadores:**
  - LED Verde (pino D5) - Status do sensor
  - LED Vermelho (pino D4) - Ativação do SKIB
  - LED Working (pino D12) - Sistema funcionando
- **SKIB** (Sistema de Recuperação - pino D7)
- **Botão Reset** (pino D3) - Reset do altímetro

### Pinout:
```
D2  -> Buzzer
D3  -> Botão Reset Altímetro
D4  -> LED Vermelho (SKIB ativo)
D5  -> LED Verde (Sensor OK)
D7  -> SKIB (Sistema de Recuperação)
D12 -> LED Working (Sistema funcionando)
```

## 3. CONFIGURAÇÕES E CONSTANTES

### Parâmetros de Voo:
```cpp
#define ASCENT_THRESHOLD 5.0              // Altitude (m) para detectar início do voo
#define DESCENT_DETECTION_THRESHOLD 5.0   // Diferença (m) para detectar descida
#define MIN_ALTITUDE_TO_RECORD 5.0        // Altitude mínima para gravar na EEPROM
```

### Armazenamento EEPROM:
```cpp
#define MAX_FLIGHT_DATA_POINTS 497        // Total de pontos de dados (1000 bytes - 6 = 994/2)
#define ALTITUDE_SCALE_FACTOR 10.0        // Fator para preservar 1 casa decimal
#define PRE_FLIGHT_QUEUE_SIZE 4           // Pontos pré-voo armazenados
```

### Intervalos de Gravação:
```cpp
#define RECORD_INTERVAL_ASCENT 50         // Gravação a cada 50ms na subida
#define RECORD_INTERVAL_DESCENT 200       // Gravação a cada 200ms na descida
```

### Controle de Tempo para Leitura do Sensor:
```cpp
#define SENSOR_READ_INTERVAL 50           // Intervalo de leitura do sensor em ms
```

## 4. SISTEMA DE ESTADOS

### Estados do Sistema:
```cpp
enum FlightState {
  PRE_FLIGHT,    // Pré-voo: monitorando, não gravando
  ASCENT,        // Ascensão: gravação de alta frequência
  RECOVERY       // Recuperação: SKIB ativo, baixa frequência
};
```

### Transições de Estado:

#### PRE_FLIGHT → ASCENT:
- **Condição:** `smoothedAltitude >= ASCENT_THRESHOLD` (5 metros)
- **Ações:**
  - Salva os 4 pontos pré-voo na EEPROM
  - Inicia gravação contínua (50ms)
  - Muda frequência do buzzer

#### ASCENT → RECOVERY:
- **Condição:** `altitude_atual <= altitude_máxima - 5m` E `armado == true`
- **Ações:**
  - Ativa SKIB
  - Salva altitude máxima
  - Buzzer de emergência (4 segundos)
  - Reduz frequência de gravação (200ms)

## 5. SISTEMA DE ARMAZENAMENTO

### Layout da EEPROM:
```
Endereço 0-1:   Altitude máxima (short int, 2 bytes)
Endereço 2-5:   Pressão baseline (double, 4 bytes)  
Endereço 6+:    Dados de voo (short int, 2 bytes por ponto)
```

### Fila Circular Pré-Voo:
- **Tamanho:** 4 pontos
- **Função:** Armazena continuamente as últimas altitudes
- **Uso:** Salvos na EEPROM quando voo é detectado

### Compressão de Dados:
- Altitudes são multiplicadas por 10 e armazenadas como `short int`
- **Faixa:** -3276.8m a +3276.7m com precisão de 0.1m
- **Economia:** 50% do espaço comparado a `double`

## 6. SISTEMA DE FEEDBACK

### Frequências do Buzzer por Estado:

| Estado | Frequência | Intervalo | Descrição |
|--------|------------|-----------|-----------|
| PRE_FLIGHT | 1500 Hz | 3000ms | Tom grave - preparação |
| ASCENT | 2093 Hz | 1000ms | Tom médio - voo ativo |
| RECOVERY | 3136 Hz | 500ms | Tom agudo - emergência |
| PÓS-SKIB | 2637 Hz | 500ms | Tom localização |

### LEDs Indicadores:

| LED | Estado | Significado |
|-----|--------|-------------|
| Verde | Ligado | Sensor BMP180 funcionando |
| Vermelho | Ligado | SKIB ativado |
| Working | Piscando | Sistema operacional (1s ligado, 100ms aceso) |

## 7. FLUXO PRINCIPAL DO PROGRAMA

### Setup():
1. Inicialização da comunicação serial (9600 baud)
2. Configuração dos pinos
3. Inicialização do sensor BMP180
4. Leitura da baseline da EEPROM
5. Reset da baseline com pressão atual
6. Inicialização da fila pré-voo

### Loop():
```
1. Leitura da pressão → Cálculo da altitude → Suavização
2. Atualização do histórico de altitudes
3. Processamento baseado no estado atual:
   - PRE_FLIGHT: Adiciona à fila, não grava EEPROM
   - ASCENT: Grava EEPROM alta frequência
   - RECOVERY: Grava EEPROM baixa frequência
4. Verificação de transições de estado
5. Atualização de altitude máxima
6. Gerenciamento de buzzer e LEDs
```

## 8. ALGORITMOS PRINCIPAIS

### Suavização de Altitude:
```cpp
// Média móvel das últimas 5 leituras
double getSmoothedAltitude(double newAltitude) {
  // Adiciona nova leitura à fila circular
  // Calcula média das leituras válidas
  // Retorna altitude suavizada
}
```

### Detecção de Fases:
```cpp
// PRE_FLIGHT → ASCENT
if (smoothedAltitude >= ASCENT_THRESHOLD) {
  // Transição para ascensão
}

// ASCENT → RECOVERY  
if (altitude_atual <= altitude_max - DESCENT_DETECTION_THRESHOLD && armado) {
  // Ativação do sistema de recuperação
}
```

### Gerenciamento SKIB:
```cpp
// Ativação
void activateRecoverySystem() {
  ativaSkib();                    // Liga o SKIB
  armado = false;                 // Desarma o sistema
  currentState = RECOVERY;        // Muda estado
  // Buzzer de emergência por 4 segundos
}

// Desativação automática após 2 segundos
if (millis() - skibActivatedAt >= SKIB_DEACTIVATION_TIME) {
  desativarSkib();
}
```

## 9. TRATAMENTO DE ERROS

### Sensor BMP180:
- **Falha na inicialização:** LED vermelho, tentativas de reconexão
- **Falha na leitura:** Retorna -1, não processa dados
- **Reconexão automática:** Verifica estado a cada loop

### EEPROM:
- **Overflow:** Verifica limites antes de gravar
- **Dados inválidos:** Baseline padrão se dados corrompidos

## 10. CARACTERÍSTICAS TÉCNICAS

### Performance:
- **Taxa de amostragem máxima:** 20 Hz (50ms)
- **Resolução de altitude:** 0.1 metros
- **Capacidade de armazenamento:** 497 pontos de dados
- **Duração típica de gravação:** ~25 segundos (alta freq.) + descida

### Limitações:
- **Altitude máxima:** 3276.7 metros
- **Temperatura de operação:** Conforme especificação do BMP180
- **Delay no sensor:** ~20ms por leitura (devido às medições de temp/pressão)

### Consumo de Memória:
- **SRAM:** ~150 bytes para variáveis globais
- **EEPROM:** 1000 bytes (497 pontos × 2 bytes + 6 bytes de metadados)
- **Flash:** ~8KB estimado

## 11. CALIBRAÇÃO E TESTES

### Baseline:
- Calculada automaticamente na inicialização
- Baseada na pressão atmosférica atual
- Salva na EEPROM para persistência

### Teste de Funcionamento:
1. **LED Working piscando:** Sistema operacional
2. **LED Verde:** Sensor funcionando
3. **Buzzer periódico:** Confirma estado atual
4. **Monitor Serial:** Dados de altitude e estado em tempo real

### Verificação dos Dados:
- Use o sketch `readeeprom.ino` para ler dados gravados
- Dados incluem timestamps estimados e fases do voo
- Análise de velocidade possível com os pontos pré-voo

## 12. MANUTENÇÃO E MODIFICAÇÕES

### Parâmetros Ajustáveis:
- `ASCENT_THRESHOLD`: Sensibilidade de detecção de voo
- `DESCENT_DETECTION_THRESHOLD`: Sensibilidade de detecção de apogeu
- Frequências e intervalos de buzzer
- Intervalos de gravação na EEPROM

### Expansões Possíveis:
- Adição de acelerômetro para detecção redundante
- Interface bluetooth para configuração
- Múltiplos sistemas de recuperação
- Gravação em cartão SD para maior capacidade

## 📋 Histórico de Versões

### Versão 3.2 (Junho 2025)
- **Otimização do Loop Principal**: Substituído `delay(50)` por controle baseado em `millis()`
  - Adicionada constante `SENSOR_READ_INTERVAL = 50ms`
  - Implementado controle de tempo não-bloqueante para leitura do sensor
  - Buzzer, LEDs e outras funções agora executam continuamente sem interrupções
  - Melhora na responsividade do sistema e detecção de transições de estado

---

