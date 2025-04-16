#include <Arduino.h>
#include <AccelStepper.h>
#include <TMCStepper.h>
#include "driver/twai.h"

// Motor driver pins
#define EN_PIN           15 // Enable
#define DIR_PIN          2  // Direction
#define STEP_PIN         4  // Step
#define CS_PIN           18 // Chip select
#define SW_MOSI          21 // Software Master Out Slave In (MOSI)
#define SW_MISO          5  // Software Master In Slave Out (MISO)
#define SW_SCK           19 // Software Slave Clock (SCK)

// Encoder pins
#define ENCODER_PIN_A    26  // A+ pin
#define ENCODER_PIN_B    27  // B+ pin

#define R_SENSE 0.11f

// Limit switch pins
#define UP_LIMIT_SWITCH_PIN    35 // UP Limit Switch connected to pin 13
#define DOWN_LIMIT_SWITCH_PIN  34 // DOWN Limit Switch connected to pin 12

// Motor specifications
constexpr float steps_per_rev = 200.0 * 16;  // 200 full steps per rev * 16 microsteps
constexpr float movement_per_rev = 4.44;     // Motor moves 4.44 mm per revolution
constexpr float steps_per_mm = steps_per_rev / movement_per_rev;  // Steps per mm

// Encoder specifications
constexpr int encoder_pulses_per_rev = 1000;  // Pulses per revolution
constexpr float mm_per_pulse = movement_per_rev / encoder_pulses_per_rev;  // mm per pulse

// Motor driver and stepper setup
TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Encoder variables
volatile long encoderPosition = 0;
volatile int lastEncoded = 0;

// Calibration flag
bool isCalibrated = false;

// Timer for throttling serial print
unsigned long lastPrintTime = 0;
unsigned long printInterval = 500;  // 500 ms (0.5 seconds)

// Declare the encoder interrupt function
// void IRAM_ATTR doEncoder();

// TWAI configuration
void setupTWAI();

TaskHandle_t Task1;

void setup() {
    xTaskCreatePinnedToCore(
      SENDCAN,
      "Task1",
      10000,
      NULL,
      0,
      &Task1,
      0);

    Serial.begin(115200);
    while (!Serial);
    Serial.println("Start...");

    // Initialize SPI for TMC2130
    SPI.begin();
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    driver.begin();
    driver.rms_current(600);   // Set motor current
    driver.en_pwm_mode(1);     // Enable StealthChop
    driver.pwm_autoscale(1);
    driver.microsteps(1);      // Set to full steps (no microstepping)

    // Initialize stepper motor
    stepper.setMaxSpeed(5 * steps_per_mm);          // 5 mm/s
    stepper.setAcceleration(100 * steps_per_mm);    // 100 mm/s^2
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);     // Adjust if needed
    stepper.enableOutputs();

    // Initialize encoder pins
    pinMode(ENCODER_PIN_A, INPUT_PULLUP);
    pinMode(ENCODER_PIN_B, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A), doEncoder, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B), doEncoder, CHANGE);

    // Initialize limit switch pins
    pinMode(UP_LIMIT_SWITCH_PIN, INPUT_PULLUP);
    pinMode(DOWN_LIMIT_SWITCH_PIN, INPUT_PULLUP);

    // Initialize TWAI (CAN)
    setupTWAI();
}

void loop() {
    // Process incoming TWAI messages
  //   if (twai_receive(&rx_message, 0) == ESP_OK) {
  //   Serial.println("TWAI message received");
  // } 
  //   else {
  //   Serial.println("No TWAI message received");
  // }

    // // Run motor commands
    // if (Serial.available()) {
    //     String received = Serial.readStringUntil('\n');
    //     received.trim();
    //     Serial.println(received);

    //     if (received.equalsIgnoreCase("calibrate") || received.equalsIgnoreCase("home")) {
    //         Serial.println("Calibrate command received.");
    //         calibrateMotor();
    //     } else if (received.equalsIgnoreCase("stop")) {
    //         stepper.stop();
    //         Serial.println("Motor stopped.");
    //     } else {
    //         // Assume input is desired position in cm
    //         double desired_position_cm = received.toFloat();
    //         Serial.print("Moving to position: ");
    //         Serial.println(desired_position_cm);
    //         moveToPosition(desired_position_cm);
    //     }
    // }

    // Safety check: Stop if limit switch is triggered
    if (digitalRead(UP_LIMIT_SWITCH_PIN) == LOW && stepper.speed() > 0) {
        stepper.stop();
        Serial.println("UP Limit switch activated. Motor stopped.");
    } else if (digitalRead(DOWN_LIMIT_SWITCH_PIN) == LOW && stepper.speed() < 0) {
        stepper.stop();
        Serial.println("DOWN Limit switch activated. Motor stopped.");
    }

    // Run the stepper to execute movement
    stepper.run();

    // Print the encoder value every 500 ms
    if (millis() - lastPrintTime >= printInterval) {
        lastPrintTime = millis();
        
        // Calculate and print the current encoder position in mm
        float encoderPosition_mm = encoderPosition * mm_per_pulse;
        // Serial.print("Encoder Position (mm): ");
        // Serial.println(encoderPosition_mm / 4);
    }
}

// Function to configure TWAI (CAN)
void setupTWAI() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_17, GPIO_NUM_16, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    twai_filter_config_t f_config;
    f_config.acceptance_code = (0x00004001 << 3);
    f_config.acceptance_mask = 0;
    // f_config.acceptance_mask = 0xF;
    f_config.single_filter = true; 

    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("TWAI driver installed");
    } else {
        Serial.println("Failed to install TWAI driver");
        return;
    }

    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        Serial.println("TWAI driver started");
    } else {
        Serial.println("Failed to start TWAI driver");
    }
}

// Function to convert CAN data bytes to float
float can2length(const twai_message_t& rx_message) {
    // Convert data[1] to data[4] into a float
    union {
        uint8_t byteArray[4];
        float floatValue;
    } dataUnion;

    dataUnion.byteArray[0] = rx_message.data[1];
    dataUnion.byteArray[1] = rx_message.data[2];
    dataUnion.byteArray[2] = rx_message.data[3];
    dataUnion.byteArray[3] = rx_message.data[4];

    return dataUnion.floatValue;
}

// Function to handle TWAI messages
void handleTWAIMessage(const twai_message_t& rx_message) {
    if (rx_message.identifier == 0x4001) {
        if (rx_message.data_length_code == 8) {  // Ensure we have 8 bytes of data
            uint8_t command = rx_message.data[0];
            
            if (command == 0) {
              Serial.println("Calibrate command received.");
              calibrateMotor();
            } else if (command == 1) {
                Serial.println("Motor stopped.");
                stepper.stop();
                Serial.println("Motor stopped via TWAI.");
            } else {
                // Convert data[1] to data[4] to a float (in cm) for position
                float desired_position_cm = can2length(rx_message);
                moveToPosition(desired_position_cm);
            }
        }
    }
}

// Function to move motor to a given position in cm
void moveToPosition(double desired_position_cm) {
    // if (isCalibrated) {
        // Convert cm to mm and then to steps
        long targetPosition = desired_position_cm * 10.0 * steps_per_mm;  // Convert cm to steps
        stepper.enableOutputs();     // 10 mm/s^2
        Serial.println(targetPosition);
        stepper.moveTo(targetPosition);
        Serial.print("Moving to position (cm): ");
        Serial.println(desired_position_cm);
    // } else {
    //     Serial.println("Please calibrate the motor before setting a position.");
    // }
}

// Function to calibrate the motor
void calibrateMotor() {
    Serial.println("Starting calibration...");

    // Enable motor outputs
    stepper.enableOutputs();
    Serial.println("Motor outputs enabled.");

    // Set slow speed and acceleration for calibration
    stepper.setMaxSpeed(2 * steps_per_mm);          // 2 mm/s
    stepper.setAcceleration(10 * steps_per_mm);     // 10 mm/s^2
    Serial.println("Calibration speed and acceleration set.");

    // Move towards DOWN limit switch
    stepper.moveTo(-1000000);  // Move a large negative distance
    Serial.println("Moving towards DOWN limit switch.");

    // Run motor until limit switch is triggered
    while (digitalRead(DOWN_LIMIT_SWITCH_PIN) == HIGH) {
        stepper.run();
    }

    // Stop the motor when the limit switch is activated
    stepper.stop();
    stepper.setCurrentPosition(0);  // Set the current position to 0 (home position)
    encoderPosition = 0;  // Reset encoder value
    isCalibrated = true;

    Serial.println("Calibration complete. Motor is at home position.");

// Reset speed and acceleration after calibration
    stepper.setMaxSpeed(5 * steps_per_mm);          // 5 mm/s
    stepper.setAcceleration(100 * steps_per_mm);    // 100 mm/s^2
    Serial.println("Calibration settings reset.");
}

// Encoder interrupt routine
// void IRAM_ATTR doEncoder() {
//     int MSB = digitalRead(ENCODER_PIN_A);
//     int LSB = digitalRead(ENCODER_PIN_B);

//     int encoded = (MSB << 1) | LSB;
//     int sum = (lastEncoded << 2) | encoded;

//     // Adjust direction if necessary
//     if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) {
//         encoderPosition++;
//     } else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) {
//         encoderPosition--;
//     }

//     lastEncoded = encoded;
// }

void SENDCAN(void * parameter){
  for(;;){
    // Serial.println("receive ready");
    twai_message_t rx_message;
    if (twai_receive(&rx_message, pdMS_TO_TICKS(10)) == ESP_OK) {
      Serial.print("receive id : ");
      Serial.println(rx_message.identifier, HEX);
      // Serial.print(rx_message.data[0],HEX);
      // Serial.print(rx_message.data[1],HEX);
      // Serial.print(rx_message.data[2],HEX);
      // Serial.print(rx_message.data[3],HEX);
      // Serial.print(rx_message.data[4],HEX);
      // Serial.print(rx_message.data[5],HEX);
      // Serial.print(rx_message.data[6],HEX);
      // Serial.println(rx_message.data[7],HEX);
      handleTWAIMessage(rx_message);
    }
  //   // // Safety check: Stop if limit switch is triggered
  //   if (digitalRead(UP_LIMIT_SWITCH_PIN) == LOW && stepper.speed() > 0) {
  //       stepper.stop();
  //       Serial.println("UP Limit switch activated. Motor stopped.");
  //   } else if (digitalRead(DOWN_LIMIT_SWITCH_PIN) == LOW && stepper.speed() < 0) {
  //       stepper.stop();
  //       Serial.println("DOWN Limit switch activated. Motor stopped.");
  //   }

  //   // Run the stepper to execute movement
  //   stepper.run();

  //   // Print the encoder value every 500 ms
  //   if (millis() - lastPrintTime >= printInterval) {
  //       lastPrintTime = millis();
        
  //       // Calculate and print the current encoder position in mm
  //       float encoderPosition_mm = encoderPosition * mm_per_pulse;
  //       // Serial.print("Encoder Position (mm): ");
  //       // Serial.println(encoderPosition_mm / 4);
  //   }
  }
}