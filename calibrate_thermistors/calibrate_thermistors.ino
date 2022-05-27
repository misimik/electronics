/* Michal Mikolajczyk 2022

Use this script to measure thermistors resistances in time

*/

#include <math.h>
#include <Wire.h>
#include "ADS1X15.h"

float voltage_to_resistance( float voltage, float ref_voltage, float ref_resistance ){
  /* This is for the voltage divider.
  The voltage is measured across  the changing resistance */
  return ref_resistance / ( ref_voltage - voltage ) * voltage;
}

// Check the I2C communication
void device_check(){
  byte error;
  Serial.println("Scanning for the I2C devices...");
  Wire.beginTransmission(0x48);
  error = Wire.endTransmission();
  if (error == 0){
    Serial.printf("I2C device found at address 0x%02X\nADC1 connected\n", 0x48);
  } else if(error != 2){
    Serial.printf("Error %d at address 0x%02X\nADC1 not found\n", error, 0x48);
  }
  Wire.beginTransmission(0x49);
  error = Wire.endTransmission();
  if (error == 0){
    Serial.printf("I2C device found at address 0x%02X\nADC2 connected\n", 0x60);
  } else if(error != 2){
    Serial.printf("Error %d at address 0x%02X\nADC2 not found\n", error, 0x60);
  }
}

ADS1115 ADS1(0x48); // Initialize ADC - ADS1115
ADS1115 ADS2(0x49); // Initialize ADC - ADS1115
static float ref_volt = 3.3; // Check if the voltage is accurate. I've checked using multimeter and it was ok.

void setup() {
  // put your setup code here, to run once:
  Serial.begin( 115200 );
  Wire.begin();

  Serial.println( "Reading start" );
  device_check();

  ADS1.setGain(1); // Gain 0: +/- 6V; Gain 1: +/- 4V
  ADS2.setGain(1); // Gain 0: +/- 6V; Gain 1: +/- 4V
}

void loop() {
  // put your main code here, to run repeatedly:

// Read the voltages 
  float f1 = ADS1.toVoltage(1); // Voltage factor: 1 means that it measures the voltage has no addtional scaling.
  float f2 = ADS2.toVoltage(1); // Voltage factor: 1 means that it measures the voltage has no addtional scaling.
  float vol_0 = ADS1.readADC(0)*f1;
  float vol_1 = ADS1.readADC(1)*f1;
  float vol_2 = ADS1.readADC(2)*f1;
  float vol_3 = ADS1.readADC(3)*f1;
  float vol_4 = ADS2.readADC(0)*f2;
  float vol_5 = ADS2.readADC(1)*f2;
  float vol_6 = ADS2.readADC(2)*f2;
  float vol_7 = ADS2.readADC(3)*f2;
  
// Calculate the resistances
  float resistance_0 = voltage_to_resistance(vol_0, ref_volt, 9860 );
  float resistance_1 = voltage_to_resistance(vol_1, ref_volt, 9930 );
  float resistance_2 = voltage_to_resistance(vol_2, ref_volt, 9740 );
  float resistance_3 = voltage_to_resistance(vol_3, ref_volt, 9860 );
  float resistance_4 = voltage_to_resistance(vol_4, ref_volt, 10050 );
  float resistance_5 = voltage_to_resistance(vol_5, ref_volt, 9800 );
  float resistance_6 = voltage_to_resistance(vol_6, ref_volt, 9910 );
  float resistance_7 = voltage_to_resistance(vol_7, ref_volt, 9790 );

//  Print the results to Serial - read the data via Python script
  Serial.print("Volt0:\t"); Serial.print(vol_0,4); Serial.print('\t'); Serial.print("Resistance0:\t"); Serial.print(resistance_0,4); Serial.print('\t');
  Serial.print("Volt1:\t"); Serial.print(vol_1,4); Serial.print('\t'); Serial.print("Resistance1:\t"); Serial.print(resistance_1,4); Serial.print('\t');
  Serial.print("Volt2:\t"); Serial.print(vol_2,4); Serial.print('\t'); Serial.print("Resistance2:\t"); Serial.print(resistance_2,4); Serial.print('\t');
  Serial.print("Volt3:\t"); Serial.print(vol_3,4); Serial.print('\t'); Serial.print("Resistance3:\t"); Serial.print(resistance_3,4); Serial.print('\t');
  Serial.print("Volt4:\t"); Serial.print(vol_4,4); Serial.print('\t'); Serial.print("Resistance4:\t"); Serial.print(resistance_4,4); Serial.print('\t');
  Serial.print("Volt5:\t"); Serial.print(vol_5,4); Serial.print('\t'); Serial.print("Resistance5:\t"); Serial.print(resistance_5,4); Serial.print('\t');
  Serial.print("Volt6:\t"); Serial.print(vol_6,4); Serial.print('\t'); Serial.print("Resistance6:\t"); Serial.print(resistance_6,4); Serial.print('\t');
  Serial.print("Volt7:\t"); Serial.print(vol_7,4); Serial.print('\t'); Serial.print("Resistance7:\t"); Serial.println(resistance_7,4);

  delay(900); // Reading the data takes around 100 ms, which make this almost 1s period
  
}
