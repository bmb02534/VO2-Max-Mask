#include <Arduino.h>
#include <Wire.h>
#include <ThingSpeak.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DFRobot_OxygenSensor.h>
#include <DFRobot_SCD4X.h>
#include <Omron_D6FPH.h>
#include <math.h>

// Wifi Initialization
#define EAP_IDENTITY "bmb02534"
#define EAP_USERNAME "bmb02534"
#define EAP_PASSWORD "Allstars-13"
#define MAX_DISCONNECTS 4
const char* ssid = "PAWS-Secure";
unsigned char disconnectNum = 0;
WiFiClient client;

// ThingSpeak Config
#define THINGSPEAK_WRITE_KEY "F7ZNSHKJ1O3VVAHO"
#define CHANNEL_NAME 2935564

// Display Characteristics
#define SCREEN_WIDTH 128  // display width
#define SCREEN_HEIGHT 64  // display height
#define OLED_RESET    -1   // Reset pin

// I2C Addresses
#define Oxygen_IICAddress ADDRESS_3 // 0x73
#define Display_IICAddress 0x3C

// Initialize Sensors
DFRobot_SCD4X SCD4X(&Wire, SCD4X_I2C_ADDR); // CO2
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); // Screen
DFRobot_OxygenSensor oxygenSensor; // Oxygen
Omron_D6FPH mySensor; // Differential Pressure

// Initialize Variables
unsigned long lastUpdate = 0;
float ve = 0;
float vo2Current = 0; // units: mL/kg/min
float vo2Max = 0; // units: mL/kg/min
float FeO2;
const float FiO2 = 0.2095; // Atmospheric Oxygen % as a fraction (assumed)
const float userWeight = 86.2; // in kg ==> 190lbs
const float airDensity = 1.293; // kg/m^-3
float area;
float radius = 0.015; // 15mm
float coeffOfDischarge = 0.65; // Assumed for Sharp Edges
float flow;
float currDiffPressure;
float prevDiffPressure;
float co2PPM = 0;

// Initialize Thermistor Variables
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

// WiFi event handlers
void WiFiStationConnected(WiFiEvent_t event) {
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t event) {
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t event) {
  Serial.println("Disconnected from WiFi access point");
  Serial.print("WiFi lost connection. Trying to reconnect...");
  if (disconnectNum < MAX_DISCONNECTS) {
    Serial.print("Attempt #");
    Serial.println(disconnectNum + 1);
    WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);
    delay(30000);  // 30-second retry
  } else {
    Serial.println("Max reconnect attempts reached. Restarting...");
    ESP.restart();  // Restart the ESP32 after max retries
  }
  disconnectNum++;
}

// WiFi setup function
void wifiSetup() {
  WiFi.disconnect(true);
  WiFi.onEvent(WiFiStationConnected, ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.mode(WIFI_STA);

  Serial.println("Attempting to connect to WiFi...");
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);

  int status = WiFi.waitForConnectResult();
  if (status != WL_CONNECTED) {
    Serial.println("Connection failed, retrying...");
  } else {
    Serial.println("Connected to WiFi successfully!");
  }

  ThingSpeak.begin(client); //initialize Thingspeak connection
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

// Display Update. Shows Current VO2 and max VO2
void updateDisplay(){
  display.clearDisplay();
  Serial.println(co2PPM);
  if(co2PPM > 1500){
    display.setCursor(0, 40);
    display.print("HIGH CO2");
  }else{
    display.setCursor(0, 40);
    display.print("Temperature: ");
    display.print((floor(kalman_x * 100.0)) / 100.0);
    display.println(" F");
  }

  display.setCursor(0, 0);
  display.print("VO2: ");
  display.print(vo2Current / 10, 2);
  display.println("mL/kg/min");

  display.setCursor(0, 20);
  display.print("VO2max:");
  display.print(vo2Max / 10, 2);
  display.println(" mL/kg/min");
  display.display();
}

// Converts Differential Pressure Measurement (Pa) to flow rate
// Q = C * A * sqrt((Differential Pressure) / (rho))
void flowRate(){
  prevDiffPressure = 0;
  currDiffPressure = mySensor.getPressure();
  // Ensures nonzero pressure values
  if(currDiffPressure <= 0){
    currDiffPressure = prevDiffPressure;
  }
  // Area of circle
  area = 3.14 * radius * radius;
  float Q = coeffOfDischarge * area * sqrt(currDiffPressure / airDensity); // m^3/s
  flow = Q * 60000; // L/min
}

void gasMeasurements(){
  DFRobot_SCD4X::sSensorMeasurement_t data;
  if(SCD4X.getDataReadyStatus()){
    SCD4X.readMeasurement(&data);
    co2PPM = data.CO2ppm;
  }
  FeO2 = oxygenSensor.getOxygenData(10) / 100.0; // Oxygen % as a fraction
}

void setup(){
  Serial.begin(19200);
  while(!Serial);

  // Setup WiFi
  wifiSetup();

  // Initialize I2C
  Wire.begin();

  // Initialize each sensor individually and wait until finished until initializing next sensor
  while(!SCD4X.begin()){ // CO2
    Serial.println("CO2 Sensor Failed");
    delay(500);
  }
  while(!display.begin(SSD1306_SWITCHCAPVCC, Display_IICAddress)) { // Display
    Serial.println(F("Display Failed"));
    delay(500);
  }
  while(!mySensor.begin(MODEL_0025AD1)) {
    Serial.println("Sensor not found. Check wiring!");
    delay(500);
  }

  // CO2 Setup
  SCD4X.enablePeriodMeasure(SCD4X_STOP_PERIODIC_MEASURE);
  SCD4X.setTempComp(4.0);
  SCD4X.enablePeriodMeasure(SCD4X_START_PERIODIC_MEASURE);

  // Display Setup
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.println("Oxygen Monitor");
  display.display();
  delay(1000);

}

void loop(){
  // Track time
  unsigned long now = millis();

  // Read analog value from GPIO36
  adcValue = analogRead(36);
  rawTemp = calculateTemp(adcValue, BETA, R0, T_nominal);

  // Kalman Filter Update
  predicted_x = kalman_x;
  predicted_P = kalman_P + kalman_Q;

  kalman_gain = predicted_P / (predicted_P + kalman_R);
  kalman_x = predicted_x + kalman_gain * (rawTemp - predicted_x);
  kalman_P = (1 - kalman_gain) * predicted_P;

  // Read Differential Pressure Sensor every 100ms
  static unsigned long lastFlow = 0;
  if((now - lastFlow) >= 100){
    lastFlow = now;
    flowRate();
    vo2Current = (flow * (FiO2 - FeO2) * 1000.0) / userWeight;
    vo2Max = max(vo2Max, vo2Current);
  }

  // Updates Every 2 Seconds (Gas Sensors)
  if((now - lastUpdate) >= 2000){
    gasMeasurements();
  }

  // send data to thingspeak through the write code
  ThingSpeak.setField(1, vo2Current);
  ThingSpeak.writeFields(CHANNEL_NAME, THINGSPEAK_WRITE_KEY);
  updateDisplay();
}