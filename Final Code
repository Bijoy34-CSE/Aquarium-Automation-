// Include necessary libraries for I2C LCD and Servo
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// Pin definitions
const int tdsPin = A0;     // TDS sensor analog output pin
const int relayPin = 7;    // Relay control pin
const int trigPin = 11;    // Ultrasonic sensor trigger pin
const int echoPin = 12;    // Ultrasonic sensor echo pin
const int servoPin = 5;    // Servo motor control pin for fish feeding
const int ledPin = 9;      // LED indicator for reservoir status
const int servo360Pin = 4; // Continuous 360° servo pin for water mixing/oxygen

// TDS sensor configuration
#define TDS_VREF 5.0       // Analog reference voltage
#define TDS_TEMP 25.0      // Water temperature (°C)

// Pump thresholds with hysteresis 
#define TDS_DIRTY_THRESHOLD 120    // TDS value to turn pump ON (ppm)  
#define TDS_CLEAN_THRESHOLD 75     // TDS value to turn pump OFF (ppm)

// Ultrasonic sensor configuration
#define MIN_DISTANCE 2     // Minimum distance in cm (avoid too close readings)
#define MAX_DISTANCE 9     // Maximum distance in cm for reservoir full

// Fish Feeder Servo configuration for MG996R
#define SERVO_WAIT_TIME 60000       // Wait time between feeding cycles (1 minute in milliseconds)
#define SERVO_SPEED 100             // Servo speed (0-180, where 90 is stopped, <90 is CCW, >90 is CW)
#define FEED_CYCLES 2               // Number of complete cycles to perform
#define FEED_CYCLE_TIME 5000        // Time for one complete cycle in milliseconds (adjust for your servo speed)

// 360° Servo configuration for water mixing
#define SERVO360_RUN_TIME 120000   // Run time for 360° servo (2 minutes in milliseconds)
#define SERVO360_STOP_TIME 60000   // Stop time for 360° servo (1 minute in milliseconds)

// Variables
float tdsValue = 0;        // Actual TDS value in ppm
bool pumpState = false;    // Track whether pump is ON or OFF
unsigned long lastPumpChangeTime = 0;  // Last time pump changed state
const unsigned long MIN_PUMP_CYCLE = 5000;  // Minimum time between pump state changes (ms)
String reservoirStatus = "Unknown";  // Track reservoir status
bool ledState = false;     // Track whether LED is ON or OFF

// Fish Feeder Servo variables for MG996R
Servo feederServo;                       // Create servo object
unsigned long servoStartTime = 0;        // When current servo action started
unsigned long feedingStartTime = 0;      // When current feeding cycle started
bool isFeeding = false;                  // Whether feeder is currently active
float cyclesCompleted = 0.0;             // Track how many cycles have completed

// 360° Servo variables for water mixing
Servo servo360;                     // Create 360° servo object
unsigned long servo360StartTime = 0; // When current 360° servo period started
bool servo360Running = false;       // Track if 360° servo is running

// Define the 360° servo states for water mixing
enum Servo360State {
  RUNNING,       // Running at full speed
  STOPPED        // Stopped
};
Servo360State servo360State = STOPPED;

// Initialize the LCD with address 0x27
LiquidCrystal_I2C lcd(0x27, 16, 2);  // (I2C address, columns, rows)

// LCD update time tracking
unsigned long lastLCDUpdate = 0;
const unsigned long LCD_UPDATE_INTERVAL = 1000; // Update LCD every 1 second

void setup() {
  Serial.begin(9600);
  pinMode(tdsPin, INPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(trigPin, OUTPUT);  // Set trigPin as output
  pinMode(echoPin, INPUT);   // Set echoPin as input
  pinMode(ledPin, OUTPUT);   // Set LED pin as output
  
  // Start the waiting timers
  servoStartTime = millis();
  servo360StartTime = millis();
  
  // Start with pump OFF - Using HIGH since relay is inverted
  digitalWrite(relayPin, HIGH);  
  pumpState = false;
  
  // Start with LED OFF
  digitalWrite(ledPin, LOW);
  ledState = false;
  
  // Initialize I2C and LCD with extra delay for stability
  Wire.begin();
  delay(100);
  
  // Initialize LCD
  lcd.init();  // Initialize the LCD
  lcd.backlight();  // Turn on backlight
  
  // Display welcome message
  lcd.setCursor(0, 0);
  lcd.print("Aquarium System");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");
  
  Serial.println("Aquarium Automation System Started");
  Serial.println("--------------------------------");
  
  // Initialize servo info
  Serial.println("MG996R Feeder Configuration:");
  Serial.print("Wait time between feeds: ");
  Serial.print(SERVO_WAIT_TIME / 1000);
  Serial.println(" seconds");
  Serial.print("Servo speed: ");
  Serial.println(SERVO_SPEED);
  Serial.print("Time per cycle: ");
  Serial.print(FEED_CYCLE_TIME / 1000.0);
  Serial.println(" seconds");
  
  // Start the 360° servo (attach and start rotating)
  servo360.attach(servo360Pin);
  servo360.write(180); // Adjust speed as needed for your servo
  servo360Running = true;
  servo360State = RUNNING;
  Serial.println("360° Servo: Started initial 2-minute run");
  
  delay(2000);  // Give system time to stabilize
  
  // Clear LCD for normal operation
  lcd.clear();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read and calculate actual TDS value
  float tdsValuePPM = readTDSValue();
  
  // Read ultrasonic sensor
  float distance = readUltrasonicDistance();
  
  // Determine reservoir status based on distance
  updateReservoirStatus(distance);
  
  // Handle fish feeder servo motor rotation (exactly 2 cycles)
  updateFeederServo(currentTime);
  
  // Handle 360° servo for water mixing/oxygen
  updateServo360(currentTime);
  
  // Print current status to serial
  Serial.print("TDS Value: ");
  Serial.print(tdsValuePPM);
  Serial.println(" ppm");
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  Serial.print("Reservoir: ");
  Serial.println(reservoirStatus);
  Serial.print("Pump status: ");
  Serial.println(pumpState ? "ON" : "OFF");
  Serial.print("LED status: ");
  Serial.println(ledState ? "ON" : "OFF");
  Serial.print("Feeder Servo: ");
  
  // Print current fish feeder state
  if (isFeeding) {
    Serial.print("Feeding NOW - ");
    Serial.print(cyclesCompleted, 2);
    Serial.print(" cycles of ");
    Serial.print(FEED_CYCLES);
    Serial.print(" (");
    Serial.print((cyclesCompleted / FEED_CYCLES) * 100, 1);
    Serial.println("% complete)");
  } else {
    Serial.println("Waiting for next feeding time");
  }
  
  // Print 360° servo status
  Serial.print("360° Servo: ");
  Serial.print(servo360Running ? "ON" : "OFF");
  Serial.print(" - ");
  switch(servo360State) {
    case RUNNING:
      Serial.println("Running 360°");
      break;
    case STOPPED:
      Serial.println("Stopped");
      break;
  }
  
  // Update LCD display (only update periodically to avoid flicker)
  if (currentTime - lastLCDUpdate >= LCD_UPDATE_INTERVAL) {
    updateLCD(tdsValuePPM, pumpState, distance, currentTime);
    lastLCDUpdate = currentTime;
  }
  
  // Handle pump control with anti-cycling protection
  if (tdsValuePPM >= TDS_DIRTY_THRESHOLD && !pumpState) {
    // Check if enough time has passed since last state change
    if (currentTime - lastPumpChangeTime >= MIN_PUMP_CYCLE) {
      // Turn pump ON - Using LOW since relay is inverted
      digitalWrite(relayPin, LOW);
      pumpState = true;
      lastPumpChangeTime = currentTime;
      Serial.println("PUMP ON - Dirty water detected");
      
      // Immediately update LCD when pump state changes
      updateLCD(tdsValuePPM, pumpState, distance, currentTime);
    } else {
      Serial.println("Waiting to turn pump ON (anti-cycling protection)");
    }
  } 
  else if (tdsValuePPM <= TDS_CLEAN_THRESHOLD && pumpState) {
    // Check if enough time has passed since last state change
    if (currentTime - lastPumpChangeTime >= MIN_PUMP_CYCLE) {
      // Turn pump OFF - Using HIGH since relay is inverted
      digitalWrite(relayPin, HIGH);
      pumpState = false;
      lastPumpChangeTime = currentTime;
      Serial.println("PUMP OFF - Water is clean");
      
      // Immediately update LCD when pump state changes
      updateLCD(tdsValuePPM, pumpState, distance, currentTime);
    } else {
      Serial.println("Waiting to turn pump OFF (anti-cycling protection)");
    }
  }

  // Status message to show where we are in the threshold band
  if (tdsValuePPM > TDS_CLEAN_THRESHOLD && tdsValuePPM < TDS_DIRTY_THRESHOLD) {
    Serial.println("TDS in neutral zone - maintaining current pump state");
  }
  
  // Update LED based on reservoir status
  updateLED();
  
  Serial.println("--------------------------------");
  delay(1000);  // Check every second
}

// Function to manage the 360° servo for water mixing/oxygen
void updateServo360(unsigned long currentTime) {
  // State machine for 360° servo control
  switch(servo360State) {
    case RUNNING:
      // Running state - check if time to stop
      if (currentTime - servo360StartTime >= SERVO360_RUN_TIME) {
        // Time to stop for 1 minute
        servo360.write(90); // Stop the servo (neutral position for continuous servo)
        servo360Running = false;
        
        // Update state
        servo360State = STOPPED;
        servo360StartTime = currentTime;
        Serial.println("360° Servo: Stopped - will rest for 1 minute");
      }
      break;
      
    case STOPPED:
      // Stopped state - check if time to run
      if (currentTime - servo360StartTime >= SERVO360_STOP_TIME) {
        // Time to run for 2 minutes
        servo360.write(180); // Run at full speed (adjust as needed for your servo)
        servo360Running = true;
        
        // Update state
        servo360State = RUNNING;
        servo360StartTime = currentTime;
        Serial.println("360° Servo: Running at full speed for 2 minutes");
      }
      break;
  }
}

// Function to update LED based on reservoir status
void updateLED() {
  // Turn LED ON when reservoir is empty and OFF when full
  if (reservoirStatus == "Empty" && !ledState) {
    digitalWrite(ledPin, HIGH);
    ledState = true;
    Serial.println("LED ON - Reservoir empty!");
  } 
  else if (reservoirStatus == "Full" && ledState) {
    digitalWrite(ledPin, LOW);
    ledState = false;
    Serial.println("LED OFF - Reservoir full");
  }
}

// Function to manage the fish feeder servo for EXACTLY 2 cycles at continuous speed
void updateFeederServo(unsigned long currentTime) {
  // Check if it's time to start feeding
  if (!isFeeding && currentTime - servoStartTime >= SERVO_WAIT_TIME) {
    // Time to start feeding
    feederServo.attach(servoPin);
    isFeeding = true;
    feedingStartTime = currentTime;
    cyclesCompleted = 0.0;
    
    // Start the servo at constant speed (clockwise)
    feederServo.write(SERVO_SPEED);  // Value > 90 for clockwise rotation
    
    Serial.println("FEEDING STARTED - Will perform exactly 2 cycles");
    Serial.print("MG996R servo started at speed: ");
    Serial.println(SERVO_SPEED);
  }
  
  // Handle the active feeding state with constant speed
  if (isFeeding) {
    // Calculate how many cycles completed based on elapsed time
    unsigned long feedingElapsed = currentTime - feedingStartTime;
    cyclesCompleted = (float)feedingElapsed / FEED_CYCLE_TIME;
    
    // Debug output - print cycle status every quarter cycle
    static float lastReportedCycle = -0.25;
    if ((int)(cyclesCompleted * 4) > (int)(lastReportedCycle * 4)) {
      lastReportedCycle = cyclesCompleted;
      Serial.print("Feeding progress: ");
      Serial.print(cyclesCompleted);
      Serial.print(" cycles of ");
      Serial.println(FEED_CYCLES);
    }
    
    // Check if all cycles are complete (exactly 2 cycles)
    if (cyclesCompleted >= FEED_CYCLES) {
      // We've finished exactly 2 cycles!
      // Stop the servo
      // Brief pause to ensure stop command is received
      feederServo.detach();   // Detach to save power
      
      isFeeding = false;      // No longer feeding
      servoStartTime = currentTime;  // Reset the wait timer
      
      Serial.println("FEEDING COMPLETE - Exactly 2 cycles performed");
      Serial.println("MG996R servo stopped");
    }
  }
}

// Function to read ultrasonic sensor and get distance in cm
float readUltrasonicDistance() {
  // Clear the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Set the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Read the echoPin, return the sound wave travel time in microseconds
  long duration = pulseIn(echoPin, HIGH);
  
  // Calculate the distance
  float distance = duration * 0.034 / 2;  // Speed of sound wave divided by 2 (go and back)
  
  // Constrain distance readings to avoid erratic values
  if (distance > 400 || distance < 2) {
    return 400;  // Return max distance if out of range
  }
  
  return distance;
}

// Update reservoir status based on distance
void updateReservoirStatus(float distance) {
  if (distance <= MAX_DISTANCE) {
    reservoirStatus = "Full";
  } else {
    reservoirStatus = "Empty";
  }
}

// Function to update LCD display showing remaining time
void updateLCD(float tdsValue, bool pumpState, float distance, unsigned long currentTime) {
  // We'll rotate display between 4 states:
  // 1. TDS value and pump status
  // 2. Reservoir status
  // 3. Fish feeding servo status (pin 5)
  // 4. Water mixer/oxygen servo status (pin 4)
  static int displayState = 0;
  
  switch(displayState) {
    case 0:
      // TDS & Pump info
      lcd.setCursor(0, 0);
      lcd.print("TDS: ");
      lcd.print(tdsValue, 0);  // No decimal places
      lcd.print(" PPM    ");    // Extra spaces to clear previous content
      
      lcd.setCursor(0, 1);
      lcd.print("Pump: ");
      if (pumpState) {
        lcd.print("ON CLEANING ");
      } else {
        lcd.print("OFF        ");
      }
      break;
      
    case 1:
      // Reservoir info
      lcd.setCursor(0, 0);
      lcd.print("Reservoir     ");
      
      lcd.setCursor(0, 1);
      lcd.print("Status: ");
      lcd.print(reservoirStatus);
      if (reservoirStatus == "Empty") {
        lcd.print(" !LED!");
      } else {
        lcd.print("      ");
      }
      break;
      
    case 2:
      // Fish feeding servo info (pin 5) - MG996R
      lcd.setCursor(0, 0);
      lcd.print("Fish Feeder   ");
      
      lcd.setCursor(0, 1);
      if (isFeeding) {
        // Show progress as percentage of total cycles
        int percentComplete = (cyclesCompleted * 100) / FEED_CYCLES;
        lcd.print("Feed: ");
        lcd.print(percentComplete);
        lcd.print("% done   ");
      } else {
        // Show countdown to next feeding cycle
        unsigned long timeRemaining = 0;
        if (SERVO_WAIT_TIME > (currentTime - servoStartTime)) {
          timeRemaining = (SERVO_WAIT_TIME - (currentTime - servoStartTime)) / 1000;
        }
        lcd.print("Next: ");
        lcd.print(timeRemaining);
        lcd.print("s    ");
      }
      break;
      
    case 3:
      // Water mixer/oxygen servo info (pin 4)
      lcd.setCursor(0, 0);
      lcd.print("Water Mixer    ");
      
      lcd.setCursor(0, 1);
      switch(servo360State) {
        case RUNNING:
          {
            // Show remaining run time
            unsigned long timeRemaining = 0;
            if (SERVO360_RUN_TIME > (currentTime - servo360StartTime)) {
              timeRemaining = (SERVO360_RUN_TIME - (currentTime - servo360StartTime)) / 1000;
            }
            lcd.print("Running: ");
            lcd.print(timeRemaining);
            lcd.print("s   ");
          }
          break;
        case STOPPED:
          {
            // Show remaining stop time
            unsigned long timeRemaining = 0;
            if (SERVO360_STOP_TIME > (currentTime - servo360StartTime)) {
              timeRemaining = (SERVO360_STOP_TIME - (currentTime - servo360StartTime)) / 1000;
            }
            lcd.print("Paused: ");
            lcd.print(timeRemaining);
            lcd.print("s   ");
          }
          break;
      }
      break;
  }
  
  // Rotate to next display state
  displayState = (displayState + 1) % 4;
}

// Function to properly read and calculate TDS value in PPM
float readTDSValue() {
  // Take multiple readings for stability
  int buffer_arr[10];
  for (int i = 0; i < 10; i++) {
    buffer_arr[i] = analogRead(tdsPin);
    delay(10);
  }
  
  // Sort readings (bubble sort)
  for (int i = 0; i < 9; i++) {
    for (int j = i + 1; j < 10; j++) {
      if (buffer_arr[i] > buffer_arr[j]) {
        int temp = buffer_arr[i];
        buffer_arr[i] = buffer_arr[j];
        buffer_arr[j] = temp;
      }
    }
  }
  
  // Take average of middle 6 readings (removing 2 highest and 2 lowest)
  int avgValue = 0;
  for (int i = 2; i < 8; i++) {
    avgValue += buffer_arr[i];
  }
  avgValue = avgValue / 6;
  
  // Convert to voltage
  float voltage = avgValue * (TDS_VREF / 1024.0);
  
  // Temperature compensation
  float compensationCoefficient = 1.0 + 0.02 * (TDS_TEMP - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;
  
  // Convert voltage to TDS value
  float tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage 
                  - 255.86 * compensationVoltage * compensationVoltage 
                  + 857.39 * compensationVoltage) * 0.5;
  
  return tdsValue;
}
