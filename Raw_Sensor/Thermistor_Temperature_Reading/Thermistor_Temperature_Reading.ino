#include <math.h>

// Initialize Variables
const float V_in = 5;
float V_INA;
float V_Wheatstone;
float R0 = 1000; // Nominal Resistance @ 25C
float T_nominal = 25.0 + 273.15; // Nominal Temperature (K)
float BETA = 3636; // Thermistor Beta Value
float R_parallel = 612.37;
float R_3 = 820; // Other 3 Resistors in Wheatstone Bridge
float R_thermistor;
float R_g = 15107; // Gain Resistor on INA
float G = 1 + (50000.0 / R_g); // Gain on INA
float rawTemp;
float adcValue;

// Kalman Filter Variables
float kalman_x = 0.0;      // Initial state estimate
float kalman_P = 1.0;      // Initial estimate covariance
float kalman_Q = 0.1;      // Process noise covariance
float kalman_R = 0.1;      // Measurement noise covariance
float predicted_x;
float predicted_P;
float kalman_gain;

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(9600);  
}

float ADC_to_Voltage(float adcValue){
  return ((adcValue + 188) / 1278);
}

float INA_to_Wheatstone(float ina, float gain){
  return ((ina - 5) / gain);
}

float Wheatstone_to_Thermistor(float wheat, float Rp = R_parallel, 
                              float R3 = R_3, float Vin = V_in){
  return (((Rp*R3)*(Vin + (2*wheat))) / ((Rp*(Vin - (2*wheat))) - R3*(Vin + (2*wheat))));
}

float calculateTemp(float adcValue, float BETA, float R_nominal, float T_nominal){
  V_INA = ADC_to_Voltage(adcValue);
  V_Wheatstone = INA_to_Wheatstone(V_INA, G);
  R_thermistor = Wheatstone_to_Thermistor(V_Wheatstone);
  return (BETA / (log(R_thermistor / ((R_nominal)*exp((-BETA / T_nominal))))) - 273.15); 
}

void loop() {
  // Read analog value from GPIO36
  adcValue = analogRead(36);
  rawTemp = calculateTemp(adcValue, BETA, R0, T_nominal);

  // Print rawTemp Temperature in C
  Serial.print("Raw Temperature: ");
  Serial.println(rawTemp);

  // Kalman Filter Update
  predicted_x = kalman_x;
  predicted_P = kalman_P + kalman_Q;

  kalman_gain = predicted_P / (predicted_P + kalman_R);
  kalman_x = predicted_x + kalman_gain * (rawTemp - predicted_x);
  kalman_P = (1 - kalman_gain) * predicted_P;

  // Print filtered temperature
  Serial.print("Filtered Temperature: ");
  Serial.println(kalman_x, 2);  // 2 decimal places for readability

  // Print ADC Value
  //Serial.print("ADC Value: ");
  //Serial.println(adcValue);

  // Print ADC Voltage
  // Serial.print("ADC Voltage: ");
  // Serial.println(ADC_to_Voltage(adcValue));

  // Print Resistance of Thermistor
  // Serial.print("Thermistor Resistance: ");
  // Serial.println(Wheatstone_to_Thermistor(V_Wheatstone));

  // Print Wheatstone Bridge Output
  // Serial.print("Wheatstone Vout: ");
  // Serial.println(INA_to_Wheatstone(V_INA, G));

  // Serial.println("");

  V_INA = 0;
  V_Wheatstone = 0;
  R_thermistor = 0;

  // Delay between readings
  delay(2000); // 2 second delay
}

