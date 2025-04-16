#include <Arduino.h>
#include "driver/twai.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#define byteRead(x,n) ( * ( (unsigned char*)(&x) + n) )

uint32_t twai_error_count = 0; 
#define reset_coutoff 20

// [Contact sensor]
const int pin_ct[2] = {40, 39};
int contact[2] = {0,0};
int contact_bool[2] = {0, 0};
int count_0[2] = {0, 0};
int count_1[2] = {0, 0};

// [Linear Encoder]
const int LinearEncoder1PinA = 1;
const int LinearEncoder1PinB = 2;
const int LinearEncoder2PinA = 42;
const int LinearEncoder2PinB = 41;
volatile int LinearEncoderPulse1 = 0;
int LinearEncoderPinALast1 = LOW;
volatile int LinearEncoderPulse2 = 0;
int LinearEncoderPinALast2 = LOW;
int LinearEncoderPos1_int32 = 0;
int LinearEncoderPos2_int32 = 0;

// [MD200T driver]
const int PIN_TURRET_PWM1 = 10;
const int PIN_TURRET_PWM2 = 11;
const int PIN_TURRET_DIR1 = 12;
const int PIN_TURRET_DIR2 = 9;
const int PIN_TURRET_ON1 = 13;
const int PIN_TURRET_ON2 = 14;

int bool_turret_on1 = 1;
int bool_turret_on2 = 1;
int turret_pwm1 = 0;
int turret_pwm2 = 0;
int bool_turret_dir1 = 0;
int bool_turret_dir2 = 0;

// [Linear Encoder Interupt]
void IRAM_ATTR updateLinearEncoder1() {  //IRAM_ATTR call interupt function
  int LinearEncoderPinAState1 = digitalRead(LinearEncoder1PinA);

  if (LinearEncoderPinAState1 != LinearEncoderPinALast1) {
    if (digitalRead(LinearEncoder1PinB) != LinearEncoderPinAState1) {
      LinearEncoderPulse1++;
    } else {
      LinearEncoderPulse1--;
    }
  }

  LinearEncoderPinALast1 = LinearEncoderPinAState1;
}

void IRAM_ATTR updateLinearEncoder2() {  //IRAM_ATTR call interupt function
  int LinearEncoderPinAState2 = digitalRead(LinearEncoder2PinA);

  if (LinearEncoderPinAState2 != LinearEncoderPinALast2) {
    if (digitalRead(LinearEncoder2PinB) != LinearEncoderPinAState2) {
      LinearEncoderPulse2++;
    } else {
      LinearEncoderPulse2--;
    }
  }

  LinearEncoderPinALast2 = LinearEncoderPinAState2;
}

// [Multitask]
TaskHandle_t Task1;

void tx_to_esp();

void setup() {
  Serial.begin(115200);
  
  pinMode(pin_ct[0],INPUT_PULLUP);
  pinMode(pin_ct[1],INPUT_PULLUP);
  
  pinMode(LinearEncoder1PinA, INPUT_PULLUP);
  pinMode(LinearEncoder1PinB, INPUT_PULLUP);
  pinMode(LinearEncoder2PinA, INPUT_PULLUP);
  pinMode(LinearEncoder2PinB, INPUT_PULLUP);

//  pinMode(PIN_TURRET_PWM1, OUTPUT);
//  pinMode(PIN_TURRET_PWM2, OUTPUT);
  ledcSetup(0, 10000, 8);
  ledcSetup(1, 10000, 8);
  ledcAttachPin(PIN_TURRET_PWM1, 0);
  ledcAttachPin(PIN_TURRET_PWM2, 1);
  
  pinMode(PIN_TURRET_DIR1, OUTPUT);
  pinMode(PIN_TURRET_DIR2, OUTPUT);
  pinMode(PIN_TURRET_ON1, OUTPUT);
  pinMode(PIN_TURRET_ON2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LinearEncoder1PinA), updateLinearEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LinearEncoder2PinA), updateLinearEncoder2, CHANGE);
  
  setup_can_rx();
  
  xTaskCreatePinnedToCore(
    TASK1,         // task function
    "Task1",           // task name
    10000,             // stack size (word)
    NULL,              // task parameters
    0,                 // task priority
    &Task1,            // task handle
    0);                // exe core
}



void loop() {
  for (int i=0;i<2;i++) {
    contact_bool[i] = digitalRead(pin_ct[i]);
    if (contact_bool[i] == 1) {
      contact[i] = 0;
      count_1[i] = 0;
    }
    else {
      count_1[i]++;
      if (count_1[i] > 10) {
        contact[i] = 1;
        count_1[i] = 0;
      }
      
    }
  }
  
  vTaskDelay(pdMS_TO_TICKS(1));
  turn_turret();
}


//=============Functions==============
void setup_can_rx()
{
  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_4, GPIO_NUM_5, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    printf("Driver installed\n");
  }
  else
  {
    printf("Failed to install driver\n");
    return;
  }
  // Start TWAI driver
  if (twai_start() == ESP_OK)
  {
    printf("Driver started\n");
  }
  else
  {
    printf("Failed to start driver\n");
    return;
  }
}

void TASK1(void * parameter) {
  for(;;) {
    twai_message_t CanRecieve;
    recieve_can(CanRecieve);
    tx_CAN();
    // vTaskDelay(0.001);
  }
}

void recieve_can(twai_message_t& CanRecieve) {
  
  // if (twai_receive(&CanRecieve, pdMS_TO_TICKS(10000)) == ESP_OK)
  if (twai_receive(&CanRecieve, 0) == ESP_OK)
  {

    //printf("0x%03x : \n", CanRecieve.identifier);
 
    if (CanRecieve.identifier == 0x20){
      
    
      for (int i = 0; i < CanRecieve.data_length_code; i++)
      {
        //printf("0x%02x ", CanRecieve.data[i]);
        // message protocol: ON1, ON2, DIR1, DIR2, PWM1, PWM2, X, X
        bool_turret_dir1 = int(CanRecieve.data[0]);
        bool_turret_dir2 = int(CanRecieve.data[1]);
        turret_pwm1 = int(CanRecieve.data[2]);
        turret_pwm2 = int(CanRecieve.data[3]);
        bool_turret_on1 = int(CanRecieve.data[4]);
        bool_turret_on2 = int(CanRecieve.data[5]);


      }


    }
  }
  vTaskDelay(pdMS_TO_TICKS(2));
}


void can_contact() {
  twai_message_t CanContactSensor;
  CanContactSensor.identifier = 0x2891;
  CanContactSensor.extd = 1;
  CanContactSensor.data_length_code = 8;
  
  CanContactSensor.data[0] = byte(contact[0]);
  CanContactSensor.data[1] = byte(contact[1]);
  CanContactSensor.data[2] = 0x00;
  CanContactSensor.data[3] = 0x00;
  CanContactSensor.data[4] = 0x00;
  CanContactSensor.data[5] = 0x00;
  CanContactSensor.data[6] = 0x00;
  CanContactSensor.data[7] = 0x00;
  
//  Serial.print(contact_bool[0]);
//  Serial.print("  ");
//  Serial.print(contact_bool[1]);
//  Serial.print("  ");
//  
//  Serial.print(count_1[0]);
//  Serial.print("  ");
//  Serial.print(count_1[1]);
//  Serial.print("  ");
//
//  Serial.print("Contact sensor: ");
//  for (int i = 0; i <= 7; i++) {
//   Serial.print(CanContactSensor.data[i], HEX);
//   Serial.print(" ");
//  }
//  Serial.println(" ");
  
  if(twai_transmit(&CanContactSensor, pdMS_TO_TICKS(10)) != ESP_OK) {
    Serial.println("transmit error");
    twai_error_count++;
  }
}

void can_linear_encoder() {
  twai_message_t CanLinearEncoder;
  CanLinearEncoder.identifier = 0x2881;
  CanLinearEncoder.extd = 1;
  CanLinearEncoder.data_length_code = 8;
  LinearEncoderPos1_int32 = LinearEncoderPulse1 * 2;
  LinearEncoderPos2_int32 = LinearEncoderPulse2 * 2;
  CanLinearEncoder.data[0] = byteRead(LinearEncoderPos1_int32, 3);
  CanLinearEncoder.data[1] = byteRead(LinearEncoderPos1_int32, 2);
  CanLinearEncoder.data[2] = byteRead(LinearEncoderPos1_int32, 1);
  CanLinearEncoder.data[3] = byteRead(LinearEncoderPos1_int32, 0);
  CanLinearEncoder.data[4] = byteRead(LinearEncoderPos2_int32, 3);
  CanLinearEncoder.data[5] = byteRead(LinearEncoderPos2_int32, 2);
  CanLinearEncoder.data[6] = byteRead(LinearEncoderPos2_int32, 1);
  CanLinearEncoder.data[7] = byteRead(LinearEncoderPos2_int32, 0);
//  Serial.print("Linear Encoder: ");
//  for (int i = 0; i <= 7; i++) {
//   Serial.print(CanLinearEncoder.data[i]);
//   Serial.print(" ");
//  }
//  Serial.println(" ");
  
  if(twai_transmit(&CanLinearEncoder, pdMS_TO_TICKS(10)) != ESP_OK) {
    Serial.println("transmit error");
    twai_error_count++;
  }

}

void tx_CAN() {
  // Serial.println("Sensor CAN data send to ROS ");
  
  can_contact();
  can_linear_encoder();
  vTaskDelay(pdMS_TO_TICKS(2));
  if (twai_error_count > reset_coutoff) {
    Serial.println("twai error full -> restarting");
    twai_error_count = 0;
    ESP.restart();
  }
}

void turn_turret()
{
  digitalWrite(PIN_TURRET_DIR1,bool_turret_dir1);
  digitalWrite(PIN_TURRET_DIR2,bool_turret_dir2);
  digitalWrite(PIN_TURRET_ON1,bool_turret_on1);
  digitalWrite(PIN_TURRET_ON2,bool_turret_on2);
  ledcWrite(0,turret_pwm1);
  ledcWrite(1,turret_pwm2);
  
  Serial.print(bool_turret_dir1);
  Serial.print("  ");
  Serial.print(bool_turret_dir2);
  Serial.print("  ");
  Serial.print(turret_pwm1);
  Serial.print("  ");
  Serial.print(turret_pwm2);
  Serial.print("  ");
  Serial.print(bool_turret_on1);
  Serial.print("  ");
  Serial.print(bool_turret_on2);
  Serial.println("  ");

}
