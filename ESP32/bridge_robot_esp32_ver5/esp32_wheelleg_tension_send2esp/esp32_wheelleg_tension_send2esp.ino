#include <Arduino.h>
#include "driver/gpio.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#define byteRead(x,n) ( * ( (unsigned char*)(&x) + n) )

struct Union {
  uint8_t byteArray[8];
  float floatValue;
}Tension_1, Tension_2;

// [TWAI]
// CAN tx: 17 rx: 16
twai_message_t SensorCan1;
uint32_t twai_error_count_0 = 0; 
#define N_que 30

twai_handle_t twai_bus_0;

uint8_t tension[6];

// [Serial1] RS422 1
//define what pin rx1 tx1 is going to be
#define rx1 18                                    
#define tx1 19
unsigned long timeout = 50; //50ms

float tension_value_1 = 0.0;
int tension_value_1_int32 = 0;
float tension_value_2 = 0.0;
int tension_value_2_int32 = 0;

// [Functions]
void tx_CAN();
float receive_tension_can();
void Tension_CAN();
void tx_to_esp();

// TWAI configuration
void setupTWAI();

// [Multitask]
TaskHandle_t Task1;

void setup() {
  xTaskCreatePinnedToCore(
    SENDCAN,         // task function
    "Task1",           // task name
    10000,             // stack size (word)
    NULL,              // task parameters
    0,                 // task priority
    &Task1,            // task handle
    0);                // exe core

  Serial0.begin(115200);
  while (!Serial0);
  Serial0.println("Start...");
  Serial1.begin(115200, SERIAL_8N1, rx1, tx1);
  Serial1.setTimeout(timeout);
  
  setupTWAI();

  SensorCan1.identifier = 0x0163;  
  SensorCan1.extd = 0;
  SensorCan1.data_length_code = 8;

}

// Function to configure TWAI (CAN)
void setupTWAI() {
    // Initialize configuration structures using macro initializers
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_17, GPIO_NUM_16, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install driver for TWAI bus 0
    g_config.controller_id = 0;
    if (twai_driver_install_v2(&g_config, &t_config, &f_config, &twai_bus_0) == ESP_OK) {
        printf("Driver installed\n");
    } else {
        printf("Failed to install driver 0\n");
        ESP.restart();
        return;
    }
    // Start TWAI driver
    if (twai_start_v2(twai_bus_0) == ESP_OK) {
        printf("Driver started\n");
    } else {
        printf("Failed to start driver 0\n");
        ESP.restart();
        return;
    }
}

// Function to convert CAN data bytes to float
Union can2tension(const twai_message_t& rx_message, Union Tension) {
    // Convert data[1] to data[3] into a float

    Tension.byteArray[0] = rx_message.data[1];
    Tension.byteArray[1] = rx_message.data[2];
    Tension.byteArray[2] = rx_message.data[3];
    
    Tension.floatValue = float(Tension.byteArray[2]<<16+Tension.byteArray[1]<<8+Tension.byteArray[0]);
    return Tension;
}

// Function to handle TWAI messages
void handleTWAIMessage(const twai_message_t& rx_message) {
    if (rx_message.identifier == 0x163) {
        if (rx_message.data_length_code == 8) {  // Ensure we have 8 bytes of data
          Tension_1 = can2tension(rx_message, Tension_1);
          tension_value_1 = Tension_1.floatValue; 
        }
    }
    else if (rx_message.identifier == 0x263) {
        if (rx_message.data_length_code == 8) {  // Ensure we have 8 bytes of data
          Tension_2 = can2tension(rx_message, Tension_2);  
          tension_value_2 = Tension_2.floatValue; 
        }
    }
}

void loop() {
  // Tension_1.byteArray[0] = 0x11;
  // Tension_1.byteArray[1] = 0x22;
  // Tension_1.byteArray[2] = 0x33;
  // Tension_2.byteArray[0] = 0x44;
  // Tension_2.byteArray[1] = 0x55;
  // Tension_2.byteArray[2] = 0x66;
  tx_to_esp();

  vTaskDelay(pdMS_TO_TICKS(9));
}

void tx_to_esp() {
  String msg_to_esp = "*" + String(Tension_1.byteArray[0])+
                      "," + String(Tension_1.byteArray[1])+
                      "," + String(Tension_1.byteArray[2])+
                      "," + String(Tension_2.byteArray[0])+
                      "," + String(Tension_2.byteArray[1])+
                      "," + String(Tension_2.byteArray[2])+
                      "," + 0x0a;
  Serial0.println(msg_to_esp);
  Serial1.println(msg_to_esp);
}

//=============MULTITASK FUNCTIONS===============
void SENDCAN(void * parameter) {
  for(;;) {
    tx_CAN();
  }
}


void tx_CAN() {
  // loop for send tension can
  twai_message_t rx_message;
  if (twai_receive_v2(twai_bus_0, &rx_message, pdMS_TO_TICKS(100)) == ESP_OK) {
    Serial0.print("receive id : ");
    Serial0.println(rx_message.identifier, HEX); 
    Serial0.print(rx_message.data[0],HEX); Serial0.print(" ");
    Serial0.print(rx_message.data[1],HEX); Serial0.print(" ");
    Serial0.print(rx_message.data[2],HEX); Serial0.print(" ");
    Serial0.print(rx_message.data[3],HEX); Serial0.print(" ");
    Serial0.print(rx_message.data[4],HEX); Serial0.print(" ");
    Serial0.print(rx_message.data[5],HEX); Serial0.print(" ");
    Serial0.print(rx_message.data[6],HEX); Serial0.print(" ");
    Serial0.println(rx_message.data[7],HEX);
    handleTWAIMessage(rx_message);
  }

  Serial0.println("tension1 : "+String(tension_value_1));
  Serial0.println("tension2 : "+String(tension_value_2));

  vTaskDelay(pdMS_TO_TICKS(10));
  // if (twai_error_count_0 > N_que) {
  //   Serial0.println("twai error full -> restarting");
  //   // if (twai_initiate_recovery == ESP_OK) {
  //   //   twai_error_count = 0;
  //   //   Serial0.println("CAN initiate recovery");
  //   // }
  //   twai_error_count_0 = 0;
  //   ESP.restart();
  // }
}

void Tension_CAN() {

  SensorCan1.data[0] = 0x00;
  SensorCan1.data[1] = Tension_1.byteArray[0];
  SensorCan1.data[2] = Tension_1.byteArray[1];
  SensorCan1.data[3] = Tension_1.byteArray[2];
  SensorCan1.data[4] = 0x00;
  SensorCan1.data[5] = Tension_2.byteArray[0];
  SensorCan1.data[6] = Tension_2.byteArray[1];
  SensorCan1.data[7] = Tension_2.byteArray[2];

  Serial0.print("Tension : ");
  Serial0.println(" ");
  for (int i = 0; i <= 7; i++) {
    Serial0.print(SensorCan1.data[i], HEX);
    Serial0.print(" ");
  }
  Serial0.println(" ");
  // twai_clear_transmit_queue_v2(twai_bus_0);
  if(twai_transmit_v2(twai_bus_0, &SensorCan1, pdMS_TO_TICKS(100)) != ESP_OK) {
    Serial0.println("transmit error");
    twai_error_count_0++;
  }
  else {
    if(twai_error_count_0 > 0){
      Serial0.println("transmit success");
      twai_error_count_0--;
    }
  }

}

