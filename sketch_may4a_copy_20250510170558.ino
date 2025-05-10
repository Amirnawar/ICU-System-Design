#include <LiquidCrystal_I2C.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <DHT_U.h>
#include <Wire.h>

// 
#define DHTPIN 2      // DHT11 data pin
#define LDRPIN A0     // LDR sensor analog pin (changed to A0)
#define TRIGGER_PIN 3 // HC-SR04 trigger pin
#define ECHO_PIN 4    // HC-SR04 echo pin
#define BUZZER_PIN 5  // Buzzer pin
#define LED1_PIN 6    // LED 1 pin 
#define LED3_PIN 8    // LED 3 pin
#define LED4_PIN 9    // LED 4 pin 
#define HW139_PIN A2  // HW-139 analog pin 
#define LED5_PIN 10   // LED 5 pin
#define MQ_PIN A1     // MQ-2 analog pin 
#define FAN1_PIN 11   // 
#define FAN2_PIN 12   //
#define ALWAYS_ON_LED2 13   // Always ON LED 2

// Constants
#define NUM_READINGS 5      // Reduced number of readings for faster response
#define SETPOINT 25.0      // Target temperature in Celsius
#define HIGH_TEMP 30.0     // Temperature threshold for maximum fan speed
#define GAS_THRESHOLD 130  // Lower threshold for gas detection to make it more sensitive
#define LIGHT_SETPOINT 100.0    // Target light level

// 
DHT dht(DHTPIN, DHT11);
long duration, distance;
LiquidCrystal_I2C lcd(0x27, 16, 2);

void scanI2CDevices() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Scanning for I2C devices...");
  
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address < 16) {
        Serial.print("0");
      }
      Serial.println(address, HEX);
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found!");
  }
  Serial.println("Scan complete.\n");
}

void setup() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED3_PIN, OUTPUT);
  pinMode(LED4_PIN, OUTPUT);
  pinMode(LED5_PIN, OUTPUT);
  pinMode(FAN1_PIN, OUTPUT);
  pinMode(FAN2_PIN, OUTPUT);
  pinMode(ALWAYS_ON_LED2, OUTPUT);
  pinMode(LDRPIN, INPUT);
  pinMode(MQ_PIN, INPUT);  // Set MQ-2 pin as input
  
  digitalWrite(ALWAYS_ON_LED2, HIGH); // Always ON
  
  Serial.begin(9600);
  delay(1000);
  
  // Warm up the MQ-2 sensor
  Serial.println("MQ-2 warming up!");
  delay(5000);  // Changed to 5 seconds warm-up
  Serial.println("MQ-2 ready!");
  
  Serial.println("\nStarting setup...");
  Serial.println("Testing LDR on pin A0...");
  // Take 5 initial readings with voltage calculations
  for(int i = 0; i < 5; i++) {
    int reading = analogRead(LDRPIN);
    float voltage = (reading * 5.0) / 1023.0;
    Serial.print("Initial reading ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(reading);
    Serial.print(" (");
    Serial.print(voltage);
    Serial.println("V)");
    delay(200);
  }
  
  Wire.begin();
  scanI2CDevices();
  
  Serial.println("Initializing LCD...");
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready");
  Serial.println("LCD initialized.");
  delay(2000);
  
  dht.begin();
}

void loop() {
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    
    // Turn on LED4 if temperature is higher than setpoint, off otherwise
    if (temperature > SETPOINT) {
        digitalWrite(LED4_PIN, HIGH);
        digitalWrite(FAN1_PIN, HIGH); // Fan ON
    } else {
        digitalWrite(LED4_PIN, LOW);
        digitalWrite(FAN1_PIN, LOW); // Fan OFF
    }
    
    // Debug print for temperature and fan
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");
    Serial.print("Fan: "); Serial.println((temperature > SETPOINT) ? "ON" : "OFF");
    
    // Direct gas sensor reading for immediate response
    int gas = analogRead(MQ_PIN);
    
    // Debug print for gas sensor
    Serial.print("Gas Level: "); 
    Serial.print(gas);
    Serial.print(" (Threshold: "); 
    Serial.print(GAS_THRESHOLD);
    Serial.println(")");
    Serial.println("------------------------");
    
    // Ultrasonic distance measurement
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
    
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration * 0.034) / 2;  // Calculate distance in cm
    
    // Check if object is too close (threshold at 15cm)
    if (distance < 15) {
        digitalWrite(LED3_PIN, HIGH);  // Turn on door LED
        Serial.println("Object detected too close!");
    } else {
        digitalWrite(LED3_PIN, LOW);   // Turn off door LED
    }
    
    // Print distance for debugging
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.println(" cm");
    
    // Touch sensor reading and response
    int touchValue = analogRead(HW139_PIN);
    const int TOUCH_THRESHOLD = 500;  // Adjust this threshold based on your sensor's sensitivity
    
    if (touchValue > TOUCH_THRESHOLD) {
        digitalWrite(LED5_PIN, HIGH);  // Turn on LED5 when touched
        Serial.println("Touch detected!");
        Serial.print("Touch value: ");
        Serial.println(touchValue);
    } else {
        digitalWrite(LED5_PIN, LOW);   // Turn off LED5 when not touched
    }
    
    // Gas detection threshold and alert system
    if (gas > GAS_THRESHOLD) {
        digitalWrite(LED1_PIN, HIGH);   // Turn on warning LED
        digitalWrite(BUZZER_PIN, HIGH); // Turn on buzzer
        digitalWrite(FAN2_PIN, HIGH);   // Turn on gas fan
    } else {
        digitalWrite(LED1_PIN, LOW);    // Turn off warning LED
        digitalWrite(BUZZER_PIN, LOW);  // Turn off buzzer
        digitalWrite(FAN2_PIN, LOW);    // Turn off gas fan
    }
    
    // Get LDR reading and print debug info
    int ldrValue = analogRead(LDRPIN);  // Direct reading
    Serial.println("\n=== Light Control Debug ===");
    Serial.print("Raw LDR Value: ");
    Serial.println(ldrValue);
    // No intensity control, LED2 is always ON
    
    // Rest of your existing code for temperature, etc.
    Serial.println("\n--- Other Sensor Readings ---");
    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");
    Serial.print("Target Temp: "); Serial.print(SETPOINT); Serial.println(" C");
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");
    
    // Display on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:"); 
    lcd.print(temperature, 1); 
    lcd.print("C ");
    lcd.print("L:ON");
    
    lcd.setCursor(0, 1);
    lcd.print("H:"); 
    lcd.print(humidity, 1); 
    lcd.print("% F:");
    lcd.print((temperature > SETPOINT) ? "ON" : "OFF");
    
    delay(2000);
}