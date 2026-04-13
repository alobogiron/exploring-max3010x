#include <MAX3010x.h>

// Sensor MAX30102
MAX30102 sensor;

// ============================
// Parametros ajustaveis
// ============================

// Serial
const unsigned long kSerialBaudRate = 115200;
const bool kPrintHeader = true;
const char kCsvSeparator = ',';

// Tempo na saida
// Aceita: false (millis), true (micros)
const bool kUseMicrosTimestamp = false;    // false: millis(); true: micros()
// Aceita: 0..4294967295 ms (0 = imprime toda amostra)
const unsigned long kPrintEveryMs = 0;     // 0 = imprime toda amostra; >0 = limita taxa de saida

// I2C
// Aceita qualquer valor uint32_t. Mais comuns: 100000 (100 kHz) e 400000 (400 kHz)
const uint32_t kWireClockHz = 400000;      // 100000 ou 400000

// Configuracao do MAX30102
// Aceita: sensor.MODE_HR_ONLY (somente RED), sensor.MODE_SPO2 (RED + IR), sensor.MODE_MULTI_LED
const auto kMode = sensor.MODE_SPO2;       // IR + RED
// Aceita:
// sensor.SAMPLING_RATE_50SPS
// sensor.SAMPLING_RATE_100SPS
// sensor.SAMPLING_RATE_200SPS
// sensor.SAMPLING_RATE_400SPS
// sensor.SAMPLING_RATE_800SPS
// sensor.SAMPLING_RATE_1000SPS
// sensor.SAMPLING_RATE_1600SPS
// sensor.SAMPLING_RATE_3200SPS
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
// Aceita:
// sensor.RESOLUTION_15BIT_69US
// sensor.RESOLUTION_16BIT_118US
// sensor.RESOLUTION_17BIT_215US
// sensor.RESOLUTION_18BIT_4110US
const auto kResolution = sensor.RESOLUTION_18BIT_4110US;
// Aceita:
// sensor.ADC_RANGE_2048NA
// sensor.ADC_RANGE_4096NA
// sensor.ADC_RANGE_8192NA
// sensor.ADC_RANGE_16384NA
const auto kAdcRange = sensor.ADC_RANGE_16384NA;
// Aceita:
// sensor.SMP_AVE_NONE (alias: sensor.SMP_AVE_1)
// sensor.SMP_AVE_2
// sensor.SMP_AVE_4
// sensor.SMP_AVE_8
// sensor.SMP_AVE_16
// sensor.SMP_AVE_32
const auto kSampleAveraging = sensor.SMP_AVE_4;
// Aceita: 0..255 (passos de 0.2 mA, 0=0 mA, 255=51.0 mA)
const uint8_t kRedCurrent = 90;            // 18.0 mA (passos de 0.2 mA)
// Aceita: 0..255 (passos de 0.2 mA, 0=0 mA, 255=51.0 mA)
const uint8_t kIrCurrent = 80;             // 16.0 mA (passos de 0.2 mA)
// Aceita: true (habilita) ou false (desabilita)
const bool kEnableFifoRollover = true;

// Leitura
// Aceita: 0..32767 ms (int)
const int kReadTimeoutMs = 200;

unsigned long lastPrintMs = 0;

void setup() {
  Serial.begin(kSerialBaudRate);

  Wire.begin();
  Wire.setClock(kWireClockHz);

  if(!sensor.begin()) {
    Serial.println("Sensor MAX30102 nao encontrado");
    while(1);
  }

  if(!sensor.setMode(kMode) ||
     !sensor.setSamplingRate(kSamplingRate) ||
     !sensor.setResolution(kResolution) ||
     !sensor.setADCRange(kAdcRange) ||
     !sensor.setSampleAveraging(kSampleAveraging) ||
     !sensor.setLedCurrent(sensor.LED_RED, kRedCurrent) ||
     !sensor.setLedCurrent(sensor.LED_IR, kIrCurrent)) {
    Serial.println("Falha na configuracao do sensor");
    while(1);
  }

  if(kEnableFifoRollover) sensor.enableFIFORollover();
  else sensor.disableFIFORollover();

  if(kPrintHeader) {
    Serial.print("tempo");
    Serial.print(kCsvSeparator);
    Serial.print("ir");
    Serial.print(kCsvSeparator);
    Serial.println("red");
  }
}

void loop() {
  auto sample = sensor.readSample(kReadTimeoutMs);
  if(!sample.valid) {
    return;
  }

  unsigned long nowMs = millis();
  if(kPrintEveryMs > 0 && (nowMs - lastPrintMs) < kPrintEveryMs) {
    return;
  }

  unsigned long t = kUseMicrosTimestamp ? micros() : nowMs;

  Serial.print(t);
  Serial.print(kCsvSeparator);
  Serial.print(sample.ir);
  Serial.print(kCsvSeparator);
  Serial.println(sample.red);

  lastPrintMs = nowMs;
}
