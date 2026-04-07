#include <MAX3010x.h>
#include "filters.h"

// Sensor MAX30105
MAX30105 sensor;

// Configuração de aquisição para MAX30105
const uint32_t kWireClockHz = 100000;
const auto kMode = sensor.MODE_SPO2;
const auto kSamplingRate = sensor.SAMPLING_RATE_400SPS;
const auto kResolution = sensor.RESOLUTION_18BIT_4110US;
const auto kAdcRange = sensor.ADC_RANGE_16384NA;
const auto kSampleAveraging = sensor.SMP_AVE_4;
const uint8_t kRedCurrent = 90;      // 18.0 mA
const uint8_t kIrCurrent = 80;       // 16.0 mA
const uint8_t kGreenCurrent = 0;     // 0.0 mA
const bool kEnableFifoRollover = true;
const float kSamplingFrequency = 400.0;

// Finger Detection Threshold and Cooldown
const unsigned long kFingerThreshold = 10000;
const unsigned int kFingerCooldownMs = 500;

// Edge Detection Threshold (decrease for MAX30100)
const float kEdgeThreshold = -2000.0;

// Filters
const float kLowPassCutoff = 5.0;
const float kHighPassCutoff = 0.5;

// Averaging
const bool kEnableAveraging = true;
const int kAveragingSamples = 50;
const int kSampleThreshold = 5;

void setup() {
  Serial.begin(115200);

  // Configura I2C para 100 kHz
  Wire.begin();
  Wire.setClock(kWireClockHz);

  if(!sensor.begin()) {
    Serial.println("Sensor not found");  
    while(1);
  }

  if(!sensor.setMode(kMode) ||
     !sensor.setSamplingRate(kSamplingRate) ||
     !sensor.setResolution(kResolution) ||
     !sensor.setADCRange(kAdcRange) ||
     !sensor.setSampleAveraging(kSampleAveraging) ||
     !sensor.setLedCurrent(sensor.LED_RED, kRedCurrent) ||
     !sensor.setLedCurrent(sensor.LED_IR, kIrCurrent) ||
     !sensor.setLedCurrent(sensor.LED_GREEN, kGreenCurrent)) {
    Serial.println("Sensor configuration failed");
    while(1);
  }

  if(kEnableFifoRollover) sensor.enableFIFORollover();
  else sensor.disableFIFORollover();

  Serial.println("Sensor initialized (MAX30105)");
}

// Filter Instances
HighPassFilter high_pass_filter(kHighPassCutoff, kSamplingFrequency);
LowPassFilter low_pass_filter(kLowPassCutoff, kSamplingFrequency);
Differentiator differentiator(kSamplingFrequency);
MovingAverageFilter<kAveragingSamples> averager;

// Timestamp of the last heartbeat
long last_heartbeat = 0;

// Timestamp for finger detection
long finger_timestamp = 0;
bool finger_detected = false;

// Last diff to detect zero crossing
float last_diff = NAN;
bool crossed = false;
long crossed_time = 0;

void loop() {
  auto sample = sensor.readSample(1000);
  float current_value = sample.red;
  
  // Detect Finger using raw sensor value
  if(sample.red > kFingerThreshold) {
    if(millis() - finger_timestamp > kFingerCooldownMs) {
      finger_detected = true;
    }
  }
  else {
    // Reset values if the finger is removed
    differentiator.reset();
    averager.reset();
    low_pass_filter.reset();
    high_pass_filter.reset();
    
    finger_detected = false;
    finger_timestamp = millis();
  }

  if(finger_detected) {
    current_value = low_pass_filter.process(current_value);
    current_value = high_pass_filter.process(current_value);
    float current_diff = differentiator.process(current_value);

    // Valid values?
    if(!isnan(current_diff) && !isnan(last_diff)) {
      
      // Detect Heartbeat - Zero-Crossing
      if(last_diff > 0 && current_diff < 0) {
        crossed = true;
        crossed_time = millis();
      }
      
      if(current_diff > 0) {
        crossed = false;
      }
  
      // Detect Heartbeat - Falling Edge Threshold
      if(crossed && current_diff < kEdgeThreshold) {
        if(last_heartbeat != 0 && crossed_time - last_heartbeat > 300) {
          // Show Results
          int bpm = 60000/(crossed_time - last_heartbeat);
          if(bpm > 50 && bpm < 250) {
            // Average?
            if(kEnableAveraging) {
              int average_bpm = averager.process(bpm);
  
              // Show if enough samples have been collected
              if(averager.count() > kSampleThreshold) {
                Serial.print("Heart Rate (avg, bpm): ");
                Serial.println(average_bpm);
              }
            }
            else {
              Serial.print("Heart Rate (current, bpm): ");
              Serial.println(bpm);  
            }
          }
        }
  
        crossed = false;
        last_heartbeat = crossed_time;
      }
    }

    last_diff = current_diff;
  }
}
