// -------------- CONFIGURATION SECTION --------------

// Output pin, MUST be 9 or 10!
#define EGT_OUTPUT  9

// Minimum and maximum temperatures in degC. Narrowing the range increases resolution and accuracy.
// This range MUST match the range set up in the ECU or the reading will be incorrect!
// Valid values are -100 to 1350
#define EGT_SENSOR_MIN_TEMP_C 0
#define EGT_SENSOR_MAX_TEMP_C 1100

// Special pins for SPI. Any pins may be used.
#define SPI_CHIPSELECT   2
#define SPI_DRDY         3

// ---------------------------------------------------

#include <Adafruit_MAX31856.h>

Adafruit_MAX31856 egtSensor(SPI_CHIPSELECT);  // set up sensor with hardware SPI

void setup() {

  Serial.begin(115200);

  // check for configuration issues
  bool configErrorDetected = false;
  if (EGT_OUTPUT != 9 && EGT_OUTPUT != 10) { configErrorDetected = true; }  // Output pin error
  if (EGT_SENSOR_MIN_TEMP_C > EGT_SENSOR_MAX_TEMP_C) { configErrorDetected = true; }  // Range error
  if ((EGT_SENSOR_MIN_TEMP_C < -100 || EGT_SENSOR_MIN_TEMP_C > 1350) || 
      (EGT_SENSOR_MAX_TEMP_C > 1350 || EGT_SENSOR_MAX_TEMP_C < -100)) { configErrorDetected = true; }  // Range out-of-bounds
  
  if (configErrorDetected == true) {
    while (true) {
      
      if (EGT_OUTPUT != 9 && EGT_OUTPUT != 10) {
        Serial.println("Config Fault: Incorrect Output Pin");
        egtErrorPulse(1001);
      }
      
      if (EGT_SENSOR_MIN_TEMP_C > EGT_SENSOR_MAX_TEMP_C) {
        Serial.println("Config Fault: Min Output Range > Max Output Range");
        egtErrorPulse(1001);
      }

      if ((EGT_SENSOR_MIN_TEMP_C < -100 || EGT_SENSOR_MIN_TEMP_C > 1350) || 
      (EGT_SENSOR_MAX_TEMP_C > 1350 || EGT_SENSOR_MAX_TEMP_C < -100)) {
        Serial.println("Config Fault: Output Range Out Of Bounds");
        egtErrorPulse(1001);
      }
    }
  }
  
  setupPWM16();  // set up 16 bit pwm
  pinMode(SPI_DRDY, INPUT);

  // attempt to init thermocouple
  if (!egtSensor.begin()) {
    while (true) {
      egtErrorPulse(901);
      Serial.println("Thermocouple Init Fault");
    }
  }

  egtSensor.setThermocoupleType(MAX31856_TCTYPE_K);  // set type K thermocouple
  egtSensor.setConversionMode(MAX31856_CONTINUOUS);  // set continuous sampling mode
}

void loop() {

  while (digitalRead(SPI_DRDY)) { ; }  // do nothing while waiting for new data
  
  double egtDegC = egtSensor.readThermocoupleTemperature();  // read temp from sensor in degC

  // check for faults
  uint8_t fault = egtSensor.readFault();
  if (fault) {
    if (fault & MAX31856_FAULT_CJRANGE) {egtErrorPulse(101); Serial.println("Cold Junction Range Fault");}
    if (fault & MAX31856_FAULT_TCRANGE) {egtErrorPulse(201); Serial.println("Thermocouple Range Fault");}
    if (fault & MAX31856_FAULT_CJHIGH)  {egtErrorPulse(301); Serial.println("Cold Junction High Fault");}
    if (fault & MAX31856_FAULT_CJLOW)   {egtErrorPulse(401); Serial.println("Cold Junction Low Fault");}
    if (fault & MAX31856_FAULT_TCHIGH)  {egtErrorPulse(501); Serial.println("Thermocouple High Fault");}
    if (fault & MAX31856_FAULT_TCLOW)   {egtErrorPulse(601); Serial.println("Thermocouple Low Fault");}
    if (fault & MAX31856_FAULT_OVUV)    {egtErrorPulse(701); Serial.println("Over/Under Voltage Fault");}
    if (fault & MAX31856_FAULT_OPEN)    {egtErrorPulse(801); Serial.println("Thermocouple Open Fault");}
  } else { setEgtOutput(egtDegC); Serial.println(egtDegC); }  // no fault. set as normal.
}

void setEgtOutput(double degreesC) {

  // temperature limits
  if (degreesC < EGT_SENSOR_MIN_TEMP_C) {
    degreesC = EGT_SENSOR_MIN_TEMP_C;
  }

  if (degreesC > EGT_SENSOR_MAX_TEMP_C) {
    degreesC = EGT_SENSOR_MAX_TEMP_C;
  }
  
  analogWrite16(EGT_OUTPUT, map(degreesC, EGT_SENSOR_MIN_TEMP_C, EGT_SENSOR_MAX_TEMP_C, 0, 65535));
}

// pulsing routine used for fault diagnosis
// half-second at 0 degrees, then 2 seconds of fault code
// repeated 3 times
void egtErrorPulse(double degreesC) {
  for (int i = 0; i < 3; i++) {
    setEgtOutput(0);
    delay(500);
    setEgtOutput(degreesC);
    delay(2000);
  }
  setEgtOutput(0);
}

void setupPWM16() {
  DDRB  |= _BV(PB1) | _BV(PB2);
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);                 
  TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
  ICR1 = 0xffff;
}

void analogWrite16(uint8_t pin, uint16_t val) {  // only works for pins 9, 10
  switch (pin) {
    case  9: OCR1A = val; break;
    case 10: OCR1B = val; break;
  }
}
