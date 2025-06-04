# Diagrama de Estados e Fluxos - RecHugo

## DIAGRAMA DE ESTADOS

```
                    INICIALIZAÇÃO
                         |
                         v
              ┌─────────────────────┐
              │     PRE_FLIGHT      │
              │                     │
              │ • Monitora altitude │
              │ • Bipa 1500Hz/3s    │
              │ • Não grava EEPROM  │
              │ • Fila circular     │
              └─────────────────────┘
                         │
                         │ altitude >= 5m
                         v
              ┌─────────────────────┐
              │       ASCENT        │
              │                     │
              │ • Grava EEPROM/50ms │
              │ • Bipa 2093Hz/1s    │
              │ • Salva pré-voo     │
              │ • Alta frequência   │
              └─────────────────────┘
                         │
                         │ altitude <= (max - 5m)
                         v
              ┌─────────────────────┐
              │      RECOVERY       │
              │                     │
              │ • Ativa SKIB        │
              │ • Grava EEPROM/200ms│
              │ • Bipa 3136Hz/500ms │
              │ • Buzzer emergência │
              └─────────────────────┘
```

## FLUXO DE DADOS

### 1. Aquisição de Dados
```
Sensor BMP180 → Pressão → Altitude Raw → Suavização → Altitude Final
                                              ↓
                                      Histórico (5 pontos)
```

### 2. Armazenamento Pré-Voo
```
Altitude Suavizada → Fila Circular (4 pontos) → Espera trigger
                                                      ↓
                                              Salva na EEPROM
```

### 3. Gravação na EEPROM
```
PRE_FLIGHT: Não grava (só fila)
     ↓
ASCENT: Fila + dados novos (50ms)
     ↓  
RECOVERY: Continua gravação (200ms)
```

## TIMELINE TÍPICO DE VOO

```
Tempo (s) │ Estado     │ Ação                    │ Buzzer    │ EEPROM
─────────────────────────────────────────────────────────────────────
0-10      │ PRE_FLIGHT │ Monitoramento           │ 1500Hz/3s │ Não
10.0      │ ASCENT     │ Detecção voo → trigger  │ 2093Hz/1s │ Inicia
10.0-15.5 │ ASCENT     │ Gravação alta freq.     │ 2093Hz/1s │ 50ms
15.5      │ RECOVERY   │ Apogeu → SKIB ativo     │ 3136Hz/4s │ 200ms
15.5+     │ RECOVERY   │ Descida                 │ 2637Hz/500ms │ 200ms
```

## ESTRUTURA DE DADOS NA EEPROM

```
Endereço │ Tamanho │ Conteúdo              │ Formato
─────────────────────────────────────────────────────
0-1      │ 2 bytes │ Altitude máxima       │ short int × 10
2-5      │ 4 bytes │ Pressão baseline      │ double
6-7      │ 2 bytes │ Ponto pré-voo #1      │ short int × 10
8-9      │ 2 bytes │ Ponto pré-voo #2      │ short int × 10
10-11    │ 2 bytes │ Ponto pré-voo #3      │ short int × 10
12-13    │ 2 bytes │ Ponto pré-voo #4      │ short int × 10
14-15    │ 2 bytes │ Primeiro ponto voo    │ short int × 10
...      │ ...     │ Dados contínuos       │ short int × 10
998-999  │ 2 bytes │ Último ponto          │ short int × 10
```

## ANÁLISE DE PERFORMANCE

### Capacidade de Armazenamento:
- **Total disponível:** 994 bytes para dados de voo
- **Pontos de dados:** 497 pontos (994 ÷ 2 bytes)
- **Duração estimada:**
  - Ascensão (50ms): 497 × 0.05s = ~25 segundos
  - Misto (4 pré + 200 ascensão + 293 descida): ~55 segundos

### Resolução Temporal:
- **Pré-voo:** Dependente do loop (~50ms)
- **Ascensão:** 50ms (20 Hz)
- **Descida:** 200ms (5 Hz)

### Precisão:
- **Altitude:** 0.1 metros
- **Timestamp:** ±20ms (devido ao sensor)

## CÓDIGO DE EXEMPLO - LEITURA DE DADOS

```cpp
// Exemplo de como ler dados gravados
void readFlightData() {
  Serial.println("=== DADOS DO VOO ===");
  
  // Ler altitude máxima
  short int maxAlt;
  EEPROM.get(0, maxAlt);
  Serial.print("Altitude máxima: ");
  Serial.print(maxAlt / 10.0);
  Serial.println(" m");
  
  // Ler baseline
  double baseline;
  EEPROM.get(2, baseline);
  Serial.print("Pressão baseline: ");
  Serial.println(baseline);
  
  // Ler pontos de dados
  for (int i = 0; i < 20; i++) {  // Primeiros 20 pontos
    short int altitude;
    EEPROM.get(6 + (i * 2), altitude);
    
    if (altitude == 0) break;  // Fim dos dados
    
    Serial.print("Ponto ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(altitude / 10.0);
    Serial.println(" m");
  }
}
```

## TROUBLESHOOTING

### Problemas Comuns:

**1. LED Vermelho sempre ligado:**
- Sensor BMP180 não conectado
- Problema na comunicação I2C
- Verificar conexões SDA/SCL

**2. Não detecta início do voo:**
- `ASCENT_THRESHOLD` muito alto
- Sensor não calibrado corretamente
- Verificar baseline

**3. SKIB não ativa:**
- `DESCENT_DETECTION_THRESHOLD` inadequado
- Altitude máxima não registrada
- Sistema já desarmado

**4. Dados corrompidos na EEPROM:**
- Reset da EEPROM necessário
- Verificar alimentação durante gravação

### Comandos de Debug:

```cpp
// Adicionar no loop para debug
Serial.print("Estado: ");
Serial.print(getStateString());
Serial.print(" | Alt: ");
Serial.print(smoothedAltitude);
Serial.print(" | Max: ");
Serial.print(maxAltitude);
Serial.print(" | Counter: ");
Serial.println(recordCounter);
```
