#include <Adafruit_MAX31856.h>

#define EGT_OUTPUT  9  // must be 9 or 10 for 16-bit PWM

#define EGT_SENSOR_MIN_TEMP_C 0
#define EGT_SENSOR_MAX_TEMP_C 1350

#define SPI_CHIPSELECT   4
#define SPI_DRDY         8

Adafruit_MAX31856 egtSensor(SPI_CHIPSELECT);  // set up sensor with hardware SPI

void setup() {
  setupPWM16();  // set up 16 bit pwm
  pinMode(SPI_DRDY, INPUT);

  // attempt to init thermocouple
  if (!egtSensor.begin()) {
    while (true) {
      egtErrorPulse(100);  // Thermocouple Init Fault 
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
    if (fault & MAX31856_FAULT_CJRANGE) egtErrorPulse(10);  // Cold Junction Range Fault
    if (fault & MAX31856_FAULT_TCRANGE) egtErrorPulse(20);  // Thermocouple Range Fault
    if (fault & MAX31856_FAULT_CJHIGH)  egtErrorPulse(30);  // Cold Junction High Fault
    if (fault & MAX31856_FAULT_CJLOW)   egtErrorPulse(40);  // Cold Junction Low Fault
    if (fault & MAX31856_FAULT_TCHIGH)  egtErrorPulse(50);  // Thermocouple High Fault
    if (fault & MAX31856_FAULT_TCLOW)   egtErrorPulse(60);  // Thermocouple Low Fault
    if (fault & MAX31856_FAULT_OVUV)    egtErrorPulse(70);  // Over/Under Voltage Fault
    if (fault & MAX31856_FAULT_OPEN)    egtErrorPulse(80);  // Thermocouple Open Fault
  } else { setEgtOutput(egtDegC); }  // no fault. set as normal.
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
// repeated 50 times
void egtErrorPulse(double degreesC) {
  for (int i; i < 50; i++) {
    setEgtOutput(0);
    delay(500);
    setEgtOutput(degreesC);
    delay(2000);
  }
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
