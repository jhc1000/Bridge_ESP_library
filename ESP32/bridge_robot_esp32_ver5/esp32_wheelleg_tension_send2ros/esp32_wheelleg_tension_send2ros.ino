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

// [Serial1] RS422 1
//define what pin rx1 tx1 is going to be
#define rx1 18                       
#define tx1 19
unsigned long timeout = 50; //50ms

uint8_t tension[6];

float tension_value_1 = 0.0;
int tension_value_1_int32 = 0;
float tension_value_2 = 0.0;
int tension_value_2_int32 = 0;

// [Functions]
void tx_CAN();
float receive_tension_can();
void Tension_CAN();
void rx_from_esp();

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
float can2tension(const twai_message_t& rx_message, Union Tension) {
    // Convert data[1] to data[3] into a float

    Tension.byteArray[0] = rx_message.data[1];
    Tension.byteArray[1] = rx_message.data[2];
    Tension.byteArray[2] = rx_message.data[3];
    
    Tension.floatValue = float(Tension.byteArray[3]<<16+Tension.byteArray[2]<<8+Tension.byteArray[1]);
    return Tension.floatValue;
}

// Function to handle TWAI messages
void handleTWAIMessage(const twai_message_t& rx_message) {
    if (rx_message.identifier == 0x0163) {
        if (rx_message.data_length_code == 8) {  // Ensure we have 8 bytes of data
          tension_value_1 = can2tension(rx_message, Tension_1);
            
        }
    }
    if (rx_message.identifier == 0x0263) {
        if (rx_message.data_length_code == 8) {  // Ensure we have 8 bytes of data
          tension_value_2 = can2tension(rx_message, Tension_2);  
            
        }
    }
}

void loop() {
  rx_from_esp();
  // loop for receive tension can
  // twai_message_t rx_message;
  // if (twai_receive_v2(twai_bus_0, &rx_message, pdMS_TO_TICKS(100)) == ESP_OK) {
  //   Serial0.print("receive id : ");
  //   Serial0.println(rx_message.identifier, HEX);
  //   // Serial0.print(rx_message.data[0],HEX);
  //   // Serial0.print(rx_message.data[1],HEX);
  //   // Serial0.print(rx_message.data[2],HEX);
  //   // Serial0.print(rx_message.data[3],HEX);
  //   // Serial0.print(rx_message.data[4],HEX);
  //   // Serial0.print(rx_message.data[5],HEX);
  //   // Serial0.print(rx_message.data[6],HEX);
  //   // Serial0.println(rx_message.data[7],HEX);
  //   handleTWAIMessage(rx_message);
  // }

  // Serial0.println("tension1 : "+String(tension_value_1));
  // Serial0.println("tension2 : "+String(tension_value_2));

  vTaskDelay(pdMS_TO_TICKS(9));
}

void rx_from_esp() {
  if (Serial1.available()>0) {
      String msg_from_esp = Serial1.readStringUntil(0x0a);
      Serial0.println(msg_from_esp);
    if (msg_from_esp.startsWith("*")) {
      int term1 = msg_from_esp.indexOf("*");               // *
      int term2 = msg_from_esp.indexOf(",", term1 + 1);    // tension1
      int term3 = msg_from_esp.indexOf(",", term2 + 1);    // tension2
      int term4 = msg_from_esp.indexOf(",", term3 + 1);    // tension3
      int term5 = msg_from_esp.indexOf(",", term4 + 1);    // tension1
      int term6 = msg_from_esp.indexOf(",", term5 + 1);    // tension2
      int term7 = msg_from_esp.indexOf(",", term6 + 1);    // tension3
  
      String tension1 = msg_from_esp.substring(term1 + 1, term2);
      String tension2 = msg_from_esp.substring(term2 + 1, term3);
      String tension3 = msg_from_esp.substring(term3 + 1, term4);
      String tension4 = msg_from_esp.substring(term4 + 1, term5);
      String tension5 = msg_from_esp.substring(term5 + 1, term6);
      String tension6 = msg_from_esp.substring(term6 + 1, term7);
  
      tension[0] = tension1.toInt();
      tension[1] = tension2.toInt();
      tension[2] = tension3.toInt();
      tension[3] = tension4.toInt();
      tension[4] = tension5.toInt();
      tension[5] = tension6.toInt();
  
      Serial0.print("tension :"); 
      Serial0.print(tension[0], HEX);   Serial0.print(" ");
      Serial0.print(tension[1], HEX);   Serial0.print(" ");
      Serial0.print(tension[2], HEX);   Serial0.print(" ");
      Serial0.print(tension[3], HEX);   Serial0.print(" ");
      Serial0.print(tension[4], HEX);   Serial0.print(" ");
      Serial0.print(tension[5], HEX);   Serial0.println(" ");
    }
    else {
      Serial0.println("====== not Serial0 data ======");
    }
  }
  else {
    Serial0.println("==== No data Received from esp ====");
  }

}

//=============MULTITASK FUNCTIONS===============
void SENDCAN(void * parameter) {
  for(;;) {
    tx_CAN();
  }
}


void tx_CAN() {
  // loop for send tension can
  Tension_CAN();
  vTaskDelay(pdMS_TO_TICKS(9));
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
  SensorCan1.data[1] = tension[0];
  SensorCan1.data[2] = tension[1];
  SensorCan1.data[3] = tension[2];
  SensorCan1.data[4] = 0x00;
  SensorCan1.data[5] = tension[3];
  SensorCan1.data[6] = tension[4];
  SensorCan1.data[7] = tension[5];

  Serial0.print("Tension : ");
  Serial0.println(" ");
  for (int i = 0; i <= 7; i++) {
    Serial0.print(SensorCan1.data[i], HEX);
    Serial0.print(" ");
  }
  Serial0.println(" ");
  twai_clear_transmit_queue_v2(twai_bus_0);
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

