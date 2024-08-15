#include <Arduino.h>  // Include Arduino core library for platform-specific functionality

// Define the pins connected to the voltage signals
#define GENERATOR_PIN 36       // Analog input pin for generator voltage (A0 on ESP32)
#define BATTERY_PIN 39         // Analog input pin for battery voltage (A1 on ESP32)
#define GENERATOR_LAMP_PIN 22  // Digital output pin for lamp control using generator (D1 on ESP32)
#define BATTERY_LAMP_PIN 23    // Digital output pin for lamp control using battery (D2 on ESP32)
#define NOT_ENOUGH_PIN 21      // Digital output pin for LED when both generator and battery are not enough (D3 on ESP32)

// Define threshold voltages
#define Min_Voltage 2.7
#define Max_Voltage 3.5

// Define interval for checking (in milliseconds)
#define CHECK_INTERVAL 10000  // 10 seconds interval

// Function prototypes
float readVoltage(int pin);
void relayA_on();
void relayB_on();
void LED_on();
void performAction(float generatorVoltage, float batteryVoltage);

unsigned long lastCheckTime = 0;  // Variable to keep track of last check time

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Set lamp pins and not enough pin as outputs
  pinMode(GENERATOR_LAMP_PIN, OUTPUT);
  pinMode(BATTERY_LAMP_PIN, OUTPUT);
  pinMode(NOT_ENOUGH_PIN, OUTPUT);

  // Ensure all pins are initially off
  digitalWrite(GENERATOR_LAMP_PIN, LOW);
  digitalWrite(BATTERY_LAMP_PIN, LOW);
  digitalWrite(NOT_ENOUGH_PIN, LOW);
}

void loop() {
  // Get current time
  unsigned long currentTime = millis();

  // Check if it's time to perform the check
  if (currentTime - lastCheckTime >= CHECK_INTERVAL) {
    // Read voltages
    float generatorVoltage = readVoltage(GENERATOR_PIN);
    float batteryVoltage = readVoltage(BATTERY_PIN);

    // Perform action based on the voltages
    performAction(generatorVoltage, batteryVoltage);

    // Update last check time
    lastCheckTime = currentTime;
  }
}

// Function to read voltage from a specified analog pin
float readVoltage(int pin) {
  // Read ADC value and convert to voltage
  int adcValue = analogRead(pin);
  float voltage = adcValue * (3.3 / 4095.0);  // Assuming Vcc is 3.3V for ESP32

  return voltage;
}

// Function to perform action based on voltages
void performAction(float generatorVoltage, float batteryVoltage) {
  // Print voltage readings for debugging
  Serial.print("Generator Voltage: ");
  Serial.print(generatorVoltage);
  Serial.println(" V");
  Serial.print("Battery Voltage: ");
  Serial.print(batteryVoltage);
  Serial.println(" V");

  // Combined decision-making logic
  if (generatorVoltage >= Min_Voltage && batteryVoltage >= Min_Voltage) {
    relayA_on(); // Turn on lamp using the first relay
    Serial.println("Using power from generator.");
  } else if (generatorVoltage >= Min_Voltage && batteryVoltage < Min_Voltage) {
    relayA_on(); // Turn on lamp using the first relay
    Serial.println("Using power from generator.");
  } else if (generatorVoltage < Min_Voltage && batteryVoltage >= Min_Voltage) {
    relayB_on(); // Turn on lamp using the second relay
    Serial.println("Using power from battery.");
  } else {
    LED_on();    // Turn on LED indicating not enough power for lamp
    Serial.println("Both generator and battery voltages are low. Turning off both lamps.");
  }
}

void relayA_on(){
  digitalWrite(GENERATOR_LAMP_PIN, HIGH);
  digitalWrite(BATTERY_LAMP_PIN, LOW);  // Ensure only generator lamp is on
  digitalWrite(NOT_ENOUGH_PIN, LOW); 
}

void relayB_on(){
  digitalWrite(GENERATOR_LAMP_PIN, LOW);
  digitalWrite(BATTERY_LAMP_PIN, HIGH);  // Ensure only battery lamp is on
  digitalWrite(NOT_ENOUGH_PIN, LOW); 
}

void LED_on(){
  digitalWrite(GENERATOR_LAMP_PIN, LOW);
  digitalWrite(BATTERY_LAMP_PIN, LOW);  // Turn off both lamps
  digitalWrite(NOT_ENOUGH_PIN, HIGH);   // Turn on "not enough power" LED
}
