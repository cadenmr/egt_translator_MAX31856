// -------------- CONFIGURATION SECTION --------------

// Output pin, MUST be 9 or 10!
#define EGT_OUTPUT_SENSOR1  9
#define EGT_OUTPUT_SENSOR2  10

// Minimum and maximum temperatures in degC. Narrowing the range increases resolution and accuracy.
// This range MUST match the range set up in the ECU or the reading will be incorrect!
// Valid values are -100 to 1350
#define EGT_SENSOR_MIN_TEMP_C 0
#define EGT_SENSOR_MAX_TEMP_C 1100

// Special pins for SPI. Any pins may be used.
#define SPI_CHIPSELECT_SENSOR1   2
#define SPI_DRDY_SENSOR1         3
#define SPI_CHIPSELECT_SENSOR2  4
#define SPI_DRDY_SENSOR2        5

// ---------------------------------------------------

#include <Adafruit_MAX31856.h>

Adafruit_MAX31856 egtSensor1(SPI_CHIPSELECT_SENSOR1);  // set up sensor1 with hardware SPI
Adafruit_MAX31856 egtSensor2(SPI_CHIPSELECT_SENSOR2);  // set up sensor2 with hardware SPI

void setup() {

  Serial.begin(115200);

  // TODO: Reimplement this for 2x sensors
  
  // check for configuration issues
  // bool configErrorDetected = false;
  // if (EGT_OUTPUT_SENSOR1 != 9 && EGT_OUTPUT_SENSOR1 != 10) { configErrorDetected = true; }  // Output pin error
  // if (EGT_SENSOR_MIN_TEMP_C > EGT_SENSOR_MAX_TEMP_C) { configErrorDetected = true; }  // Range error
  // if ((EGT_SENSOR_MIN_TEMP_C < -100 || EGT_SENSOR_MIN_TEMP_C > 1350) || 
  //     (EGT_SENSOR_MAX_TEMP_C > 1350 || EGT_SENSOR_MAX_TEMP_C < -100)) { configErrorDetected = true; }  // Range out-of-bounds
  
  // if (configErrorDetected == true) {
  //   while (true) {
      
  //     if (EGT_OUTPUT_SENSOR1 != 9 && EGT_OUTPUT_SENSOR1 != 10) {
  //       Serial.println("Config Fault: Incorrect Output Pin");
  //       egtErrorPulse1(1001);
  //     }
      
  //     if (EGT_SENSOR_MIN_TEMP_C > EGT_SENSOR_MAX_TEMP_C) {
  //       Serial.println("Config Fault: Min Output Range > Max Output Range");
  //       egtErrorPulse1(1001);
  //     }

  //     if ((EGT_SENSOR_MIN_TEMP_C < -100 || EGT_SENSOR_MIN_TEMP_C > 1350) || 
  //     (EGT_SENSOR_MAX_TEMP_C > 1350 || EGT_SENSOR_MAX_TEMP_C < -100)) {
  //       Serial.println("Config Fault: Output Range Out Of Bounds");
  //       egtErrorPulse1(1001);
  //     }
  //   }
  // }
  
  setupPWM16();  // set up 16 bit pwm
  pinMode(SPI_DRDY_SENSOR1, INPUT);
  pinMode(SPI_DRDY_SENSOR2, INPUT);

  // attempt to init thermocouple 1
  if (!egtSensor1.begin()) {
    while (true) {
      egtErrorPulse1(901);
      Serial.println("Thermocouple 1 Init Fault");
    }
  }

// attempt to init thermocouple 2
  if (!egtSensor2.begin()) {
    while (true) {
      egtErrorPulse1(901);
      Serial.println("Thermocouple 2 Init Fault");
    }
  }

  egtSensor1.setThermocoupleType(MAX31856_TCTYPE_K);  // set type K thermocouple
  egtSensor1.setConversionMode(MAX31856_CONTINUOUS);  // set continuous sampling mode

  egtSensor2.setThermocoupleType(MAX31856_TCTYPE_K);  // set type K thermocouple
  egtSensor2.setConversionMode(MAX31856_CONTINUOUS);  // set continuous sampling mode
  
}

void loop() {

  // while (digitalRead(SPI_DRDY_SENSOR1)) { ; }  // do nothing while waiting for new data

  while (true) {  // reading loop, check if we have new data from either sensor and set it

    if (!digitalRead(SPI_DRDY_SENSOR1)) {   // if we have data ready for sensor 1

      double egtDegC_sensor1 = egtSensor1.readThermocoupleTemperature();  // read temp from sensor in degC

      // check for faults
      uint8_t fault = egtSensor1.readFault();
      if (fault) {
        if (fault & MAX31856_FAULT_CJRANGE) {setEgtOutput1(101); Serial.println("1: Cold Junction Range Fault");}
        if (fault & MAX31856_FAULT_TCRANGE) {setEgtOutput1(201); Serial.println("1: Thermocouple Range Fault");}
        if (fault & MAX31856_FAULT_CJHIGH)  {setEgtOutput1(301); Serial.println("1: Cold Junction High Fault");}
        if (fault & MAX31856_FAULT_CJLOW)   {setEgtOutput1(401); Serial.println("1: Cold Junction Low Fault");}
        if (fault & MAX31856_FAULT_TCHIGH)  {setEgtOutput1(501); Serial.println("1: Thermocouple High Fault");}
        if (fault & MAX31856_FAULT_TCLOW)   {setEgtOutput1(601); Serial.println("1: Thermocouple Low Fault");}
        if (fault & MAX31856_FAULT_OVUV)    {setEgtOutput1(701); Serial.println("1: Over/Under Voltage Fault");}
        if (fault & MAX31856_FAULT_OPEN)    {setEgtOutput1(801); Serial.println("1: Thermocouple Open Fault");}
      } else { setEgtOutput1(egtDegC_sensor1); Serial.print("1: "); Serial.println(egtDegC_sensor1); }  // no fault. set as normal.

    }
    
    if (!digitalRead(SPI_DRDY_SENSOR2)) {   // if we have data ready for sensor 2

      double egtDegC_sensor2 = egtSensor2.readThermocoupleTemperature();  // read temp from sensor in degC

      // check for faults
      uint8_t fault = egtSensor2.readFault();
      if (fault) {
        if (fault & MAX31856_FAULT_CJRANGE) {setEgtOutput2(101); Serial.println("2: Cold Junction Range Fault");}
        if (fault & MAX31856_FAULT_TCRANGE) {setEgtOutput2(201); Serial.println("2: Thermocouple Range Fault");}
        if (fault & MAX31856_FAULT_CJHIGH)  {setEgtOutput2(301); Serial.println("2: Cold Junction High Fault");}
        if (fault & MAX31856_FAULT_CJLOW)   {setEgtOutput2(401); Serial.println("2: Cold Junction Low Fault");}
        if (fault & MAX31856_FAULT_TCHIGH)  {setEgtOutput2(501); Serial.println("2: Thermocouple High Fault");}
        if (fault & MAX31856_FAULT_TCLOW)   {setEgtOutput2(601); Serial.println("2: Thermocouple Low Fault");}
        if (fault & MAX31856_FAULT_OVUV)    {setEgtOutput2(701); Serial.println("2: Over/Under Voltage Fault");}
        if (fault & MAX31856_FAULT_OPEN)    {setEgtOutput2(801); Serial.println("2: Thermocouple Open Fault");}
      } else { setEgtOutput2(egtDegC_sensor2); Serial.print("2: "); Serial.println(egtDegC_sensor2); }  // no fault. set as normal.

    } 

  }

}

// TODO: USE INTERRUPTS FOR THIS INSTEAD

void setEgtOutput1(double degreesC) {

  // temperature limits
  if (degreesC < EGT_SENSOR_MIN_TEMP_C) {
    degreesC = EGT_SENSOR_MIN_TEMP_C;
  }

  if (degreesC > EGT_SENSOR_MAX_TEMP_C) {
    degreesC = EGT_SENSOR_MAX_TEMP_C;
  }
  
  analogWrite16(EGT_OUTPUT_SENSOR1, map(degreesC, EGT_SENSOR_MIN_TEMP_C, EGT_SENSOR_MAX_TEMP_C, 0, 65535));
}

void setEgtOutput2(double degreesC) {

  // temperature limits
  if (degreesC < EGT_SENSOR_MIN_TEMP_C) {
    degreesC = EGT_SENSOR_MIN_TEMP_C;
  }

  if (degreesC > EGT_SENSOR_MAX_TEMP_C) {
    degreesC = EGT_SENSOR_MAX_TEMP_C;
  }
  
  analogWrite16(EGT_OUTPUT_SENSOR2, map(degreesC, EGT_SENSOR_MIN_TEMP_C, EGT_SENSOR_MAX_TEMP_C, 0, 65535));
}

// pulsing routine used for fault diagnosis
// half-second at 0 degrees, then 2 seconds of fault code
// repeated 3 times 
void egtErrorPulse1(double degreesC) {
  for (int i = 0; i < 3; i++) {
    setEgtOutput1(0);
    delay(500);
    setEgtOutput1(degreesC);
    delay(2000);
  }
  setEgtOutput1(0);
}

void egtErrorPulse2(double degreesC) {
  for (int i = 0; i < 3; i++) {
    setEgtOutput2(0);
    delay(500);
    setEgtOutput2(degreesC);
    delay(2000);
  }
  setEgtOutput2(0);
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
