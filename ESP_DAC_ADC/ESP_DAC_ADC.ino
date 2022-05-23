#include <math.h>
#include <Wire.h>
#include "ADS1X15.h"
#include "MCP4725.h"

float temp_to_resistance(float temp, float ref_resistance, float ref_temp, float beta){
  return ref_resistance * exp( beta * ( (1/( temp + 273. ) ) - ( 1/( ref_temp + 273. ) ) ));
}

float resistance_to_temp( float resistance, float ref_resistance, float ref_temp, float beta ){
  return 1/( 1/( ref_temp + 273. ) + 1/beta*log( resistance / ref_resistance ) ) - 273.;
}

float voltage_to_resistance( float voltage, float ref_voltage, float ref_resistance ){
  /* This is for the voltage divider.
  The voltage is measured across  the changing resistance */
  return ref_resistance / ( ref_voltage - voltage ) * voltage;
}

float resistance_to_voltage( float resistance ){
  /* This is for the Wavelength Electronics PTC10K-CH
     Here a 100 uA current is pushed through the thermistor
     and the voltage drop must be equal to the controlling voltage*/
  return resistance / 10000.;
}

void device_check(){
  byte error;
  Serial.println("Scanning for the I2C devices...");
  Wire.beginTransmission(0x48);
  error = Wire.endTransmission();
  if (error == 0){
    Serial.printf("I2C device found at address 0x%02X\nADC connected\n", 0x48);
  } else if(error != 2){
    Serial.printf("Error %d at address 0x%02X\nADC not found\n", error, 0x48);
  }
  Wire.beginTransmission(0x60);
  error = Wire.endTransmission();
  if (error == 0){
    Serial.printf("I2C device found at address 0x%02X\nDAC connected\n", 0x60);
  } else if(error != 2){
    Serial.printf("Error %d at address 0x%02X\nDAC not found\n", error, 0x60);
  }
}



ADS1115 ADS(0x48); // Initialize ADC - ADS1115
MCP4725 MCP(0x60); // Initialize DAC - MCP4725
static const int tec_pin = 33;
static bool tec_status = false;




void setup() {
  // put your setup code here, to run once:
  Serial.begin( 115200 );
  Wire.begin();
  pinMode( tec_pin, OUTPUT );

  digitalWrite( tec_pin, tec_status );
  Serial.println( "Temp 15 C" );
  Serial.print("Resistance: "); Serial.println( temp_to_resistance( 15., 10000., 25., 3450. ) );
  Serial.print("Temp: "); Serial.println( resistance_to_temp( 20000., 10000., 25., 3450. ) );
  device_check();

  ADS.setGain(1); // Gain 0: +/- 6V; Gain 1: +/- 4V
  MCP.begin();
  MCP.writeDAC( 4095/3.3 * resistance_to_voltage( temp_to_resistance( 25, 10000, 25, 3450 ) ), true );

}

void loop() {
  // put your main code here, to run repeatedly:

  tec_status = not( tec_status );
  digitalWrite( tec_pin, tec_status );

  float f = ADS.toVoltage(1); // Voltage factor: 1 means that it measures the voltage has no addtional scaling.
  float voltage_0 = ADS.readADC(0)*f;
  float voltage_1 = ADS.readADC(1)*f;
  float voltage_2 = ADS.readADC(2)*f;

  float resistance_0 = voltage_to_resistance(voltage_0, 3.3, 50000 );
  float resistance_1 = voltage_to_resistance(voltage_1, 3.3, 50000 );
  float resistance_2 = voltage_2 * 10000.;

  float temp_0 = resistance_to_temp( resistance_0, 10000, 25, 3450 );
  float temp_1 = resistance_to_temp( resistance_1, 10000, 25, 3450 );
  float temp_2 = resistance_to_temp( resistance_2, 10000, 25, 3450 );
  
  Serial.print("\tAnalog0: "); Serial.print(voltage_0,3); Serial.print('\t');
  Serial.print(resistance_0, 3); Serial.print('\t'); Serial.println(temp_0, 3); 
  
  Serial.print("\tAnalog1: "); Serial.print(voltage_1,3); Serial.print('\t');
  Serial.print(resistance_1, 3); Serial.print('\t'); Serial.println(temp_1, 3);

  Serial.print("\tAnalog2: "); Serial.print(voltage_2,3); Serial.print('\t');
  Serial.print(resistance_2, 3); Serial.print('\t'); Serial.println(temp_2, 3);

  MCP.setValue( 4095/3.3 * resistance_to_voltage( temp_to_resistance( random( 10, 80 ), 10000, 25, 3450 ) ) );

  delay(2000);
  
}
