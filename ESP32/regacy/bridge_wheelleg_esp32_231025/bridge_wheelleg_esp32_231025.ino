#include <Arduino.h>
#include "driver/twai.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#include <CAN.h>

// [TWAI]
twai_message_t CanContactSensor
twai_message_t CanRecieve
uint32_t twai_error_count = 0; 
#define reset_coutoff 20

CanContactSensor.identifier = 0x2832; //ABS
CanContactSensor.extd = 1;
CanContactSensor.data_length_code = 8;

// [Contact sensor]
const int pin_ct[2] = {40, 39};
int contact[2] = {0,0};
int contact_bool[2] = {0, 0};
int count_0[2] = {0, 0};
int count_1[2] = {0, 0};

// [Linear Encoder]
// [Rotary]
const int RotaryEncoder1PinA = 1;
const int RotaryEncoder1PinB = 2;
const int RotaryEncoder2PinA = 42;
const int RotaryEncoder2PinB = 41;
volatile int RotaryEncoderPos1 = 0;
int RotaryEncoderPinALast1 = LOW;
volatile int RotaryEncoderPos2 = 0;
int RotaryEncoderPinALast2 = LOW;
float Rotary_Encoder_pos1 = 0.0;
float Rotary_Encoder_pos2 = 0.0;

// [MD200T driver]
const int PIN_TURRET_PWM1 = 10;
const int PIN_TURRET_PWM2 = 11;
const int PIN_TURRET_DIR1 = 12;
const int PIN_TURRET_DIR2 = 9;
const int PIN_TURRET_ON1 = 13;
const int PIN_TURRET_ON2 = 14;

int bool_turret_on1 = 0;
int bool_turret_on2 = 0;
int turret_pwm1 = 0;
int turret_pwm2 = 0;
int bool_turret_dir1 = 0;
int bool_turret_dir2 = 0;

// [Rotary Encoder Interupt]
void IRAM_ATTR updateRotaryEncoder1() {  //IRAM_ATTR call interupt function
  int RotaryEncoderPinAState1 = digitalRead(RotaryEncoder1PinA);

  if (RotaryEncoderPinAState1 != RotaryEncoderPinALast1) {
    if (digitalRead(RotaryEncoder1PinB) != RotaryEncoderPinAState1) {
      RotaryEncoderPos1++;
    } else {
      RotaryEncoderPos1--;
    }
  }

  RotaryEncoderPinALast1 = RotaryEncoderPinAState1;
}

void IRAM_ATTR updateRotaryEncoder2() {  //IRAM_ATTR call interupt function
  int RotaryEncoderPinAState2 = digitalRead(RotaryEncoder2PinA);

  if (RotaryEncoderPinAState2 != RotaryEncoderPinALast2) {
    if (digitalRead(RotaryEncoder2PinB) != RotaryEncoderPinAState2) {
      RotaryEncoderPos2++;
    } else {
      RotaryEncoderPos2--;
    }
  }

  RotaryEncoderPinALast2 = RotaryEncoderPinAState2;
}

// [Multitask]
TaskHandle_t Task1;

void tx_to_esp();

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial2.begin(115200);
  Serial2.setTimeout(10);
  
  pinMode(pin_ct[0],INPUT_PULLUP);
  pinMode(pin_ct[1],INPUT_PULLUP);
  
  pinMode(RotaryEncoder1PinA, INPUT_PULLUP);
  pinMode(RotaryEncoder1PinB, INPUT_PULLUP);
  pinMode(RotaryEncoder2PinA, INPUT_PULLUP);
  pinMode(RotaryEncoder2PinB, INPUT_PULLUP);

  pinMode(PIN_TURRET_PWM1, OUTPUT);
  pinMode(PIN_TURRET_PWM2, OUTPUT);
  pinMode(PIN_TURRET_DIR1, OUTPUT);
  pinMode(PIN_TURRET_DIR2, OUTPUT);
  pinMode(PIN_TURRET_ON1, OUTPUT);
  pinMode(PIN_TURRET_ON2, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(RotaryEncoder1PinA), updateRotaryEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RotaryEncoder2PinA), updateRotaryEncoder2, CHANGE);

  setup_can_rx();

  xTaskCreatePinnedToCore(
    recieve_can,         // task function
    "Task1",           // task name
    10000,             // stack size (word)
    NULL,              // task parameters
    0,                 // task priority
    &Task1,            // task handle
    0);                // exe core
}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i=0;i<2;i++) {
    contact_bool[i] = digitalRead(pin_ct[i]);
    if (contact_bool[i] == 0) {
      contact[i] = 0;
    }
    else {
      count_1[i]++;
      if (count_1[i] > 5) {
        contact[i] = 1;
        count_1[i] = 0;
      }
    }
  }
  vTaskDelay(pdMS_TO_TICKS(1));
  vTaskDelay(pdMS_TO_TICKS(1));
  turn_turret();
}

//=============Functions==============
void setup_can_rx()
{
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_5, GPIO_NUM_4, TWAI_MODE_NORMAL);
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

void recieve_can(void * parameter) {
  if (twai_receive(&CanRecieve, pdMS_TO_TICKS(10000)) == ESP_OK)
  {
    printf("0x%03x :", message.identifier);
    if (!(message.rtr))
    {
      for (int i = 0; i < message.data_length_code; i++)
      {
        printf("0x%02x ", message.data[i]);
        // message protocol: ON1, ON2, DIR1, DIR2, PWM1, PWM2, X, X
        bool_turret_on1 = int(message.data[0]);
        bool_turret_on2 = int(message.data[1]);
        turret_pwm1 = int(message.data[2]);
        turret_pwm2 = int(message.data[3]);
        bool_turret_dir1 = int(message.data[4]);
        bool_turret_dir2 = int(message.data[5]);
      }
      printf("\n");
    }
  }
  vTaskDelay(50 / portTICK_RATE_MS);
}

void CAN_EXTsend(CanData InputCanData){
  CAN.beginExtendedPacket(InputCanData.Id);
  CAN.write(InputCanData.Data, InputCanData.length);
  CAN.endPacket();
}

void can_contact() {
  CanContactSensor.data[0] = byte(contact_bool[0]);
  CanContactSensor.data[1] = byte(contact_bool[1]);
  CAN_EXTsend(CanContactSensor);
  if(twai_transmit(&CanContactSensor, pdMS_TO_TICKS(10)) != ESP_OK) {
    Serial.println("transmit error");
    twai_error_count++;
  }
}

void tx_CAN() {
  // Serial.println("Sensor CAN data send to ROS ");
  can_contact();
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
  analogWrite(PIN_TURRET_PWM1,turret_pwm1);
  analogWrite(PIN_TURRET_PWM2,turret_pwm2);

}
