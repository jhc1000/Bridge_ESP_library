#include <Arduino.h>
#include "driver/twai.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#include <HX711.h>


#define byteRead(x,n) ( * ( (unsigned char*)(&x) + n) )

// [TWAI]
// CAN tx: 47 rx: 21
twai_message_t SensorCan1, SensorCan2, SensorCan3, SensorCan4;
uint32_t twai_error_count = 0; 
#define N_que 30

// float AbsEnc_position1 = 0.0;
// int AbsEnc_position1_int32 = 0;
// float RotEnc_position1 = 0.0;
// int RotEnc_position1_int32 = 0;
float loadcell_degree1 = 0.0;
int loadcell_degree1_int32 = 0;

// [Serial1] RS422 1
//define what pin rx2 tx2 is going to be
#define rx1 4                                        
#define tx1 5
unsigned long timeout = 50; //50ms

// [Serial2] RS422 2
//define what pin rx2 tx2 is going to be
#define rx2 6                                    
#define tx2 7

// [LoadCell]
#define calibration_factor1 -2180
#define dtout1 16  //DT 전선 연결
#define clk1 17    // SCLK 전선 연결
HX711 scale1;
float LoadCell_value1 = 0.0;

// [Encoder]
// [Rotary]
// const int RotaryEncoder1PinA = 13;
// const int RotaryEncoder1PinB = 12;

// volatile int RotaryEncoderPos1 = 0;
// int RotaryEncoderPinALast1 = LOW;
// float Rotary_Encoder_pos1 = 0.0;

// [Absolute]
// int AbsoluteEncoderPin1[10]{ 4, 5, 6, 7, 15, 18, 8, 9, 10, 11};
// int AbsoluteEncoder1[10];
// float AbsoluteEncoderPos1 = 0.0;
// float Absolute_Encoder_pos1 = 0.0;

// [RS422 abs Encoder]
struct SensorData {
  byte buffer1;
  byte buffer2;
  byte buffer3;
}data1, data2;
float RS422_Encoder_pos_1 = 0.0;
float RS422_Encoder_pos_2 = 0.0;

// // [Rotary Encoder Interupt]
// void IRAM_ATTR updateRotaryEncoder1() {  //IRAM_ATTR call interupt function
//   int RotaryEncoderPinAState1 = digitalRead(RotaryEncoder1PinA);

//   if (RotaryEncoderPinAState1 != RotaryEncoderPinALast1) {
//     if (digitalRead(RotaryEncoder1PinB) != RotaryEncoderPinAState1) {
//       RotaryEncoderPos1++;
//     } else {
//       RotaryEncoderPos1--;
//     }
//   }

//   RotaryEncoderPinALast1 = RotaryEncoderPinAState1;
// }

// [Functions]
void tx_to_esp();
float rx_from_RS422_1();
float rx_from_RS422_2();
void rx_from_loadcell();
void tx_CAN();
// void EMImu_CAN();
// void EBImu_Quaternion_CAN();
// void AbsEnc_CAN();
// void RotEnc_CAN();
void RS422Enc1_CAN();
void RS422Enc2_CAN();
void LoadCell_CAN();

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

  Serial.begin(115200);
  Serial1.begin(1000000, SERIAL_8N1, rx1, tx1);
  Serial2.begin(1000000, SERIAL_8N1, rx2, tx2);
  Serial1.setTimeout(timeout);
  Serial2.setTimeout(timeout);

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_47, GPIO_NUM_21, TWAI_MODE_NORMAL); //tx,rx
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
    while(1){
      Serial.println("install error");
      ESP.restart();
      
    }
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
    while(1){
      Serial.println("install error");
      ESP.restart();
      
    }
    return;
  }

  // pinMode(RotaryEncoder1PinA, INPUT_PULLUP);
  // pinMode(RotaryEncoder1PinB, INPUT_PULLUP);

  // pinMode(AbsoluteEncoderPin1[0], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin1[1], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin1[2], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin1[3], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin1[4], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin1[5], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin1[6], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin1[7], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin1[8], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin1[9], INPUT_PULLUP);

  // attachInterrupt(digitalPinToInterrupt(RotaryEncoder1PinA), updateRotaryEncoder1, CHANGE);
 
  scale1.begin(dtout1, clk1);
  scale1.set_scale(calibration_factor1);
  scale1.tare();

  
  SensorCan1.identifier = 0x2832; //RS 1 
  SensorCan2.identifier = 0x2852; //RS 2
  // SensorCan3.identifier = 0x2852; //RS
  SensorCan4.identifier = 0x2862; //LOAD
  SensorCan1.extd = 1;
  SensorCan2.extd = 1;
  // SensorCan3.extd = 1;
  SensorCan4.extd = 1;  
  SensorCan1.data_length_code = 8;
  SensorCan2.data_length_code = 8;
  // SensorCan3.data_length_code = 8;
  SensorCan4.data_length_code = 8;  

}

void loop() {

  // AbsoluteEncoderPos1 = 0;

  // for (int i = 1; i <= 10; i++) {
  //   AbsoluteEncoder1[i] = digitalRead(AbsoluteEncoderPin1[i]);
  // }

  // for (int i = 0; i <= 9; i++) {
  //   AbsoluteEncoderPos1 = AbsoluteEncoderPos1 + AbsoluteEncoder1[i] * pow(2, i);
  //   //Serial.print(AbsoluteEncoder1[i]);
  // }


  rx_from_loadcell();

  RS422_Encoder_pos_1 = rx_from_RS422_1();
  RS422_Encoder_pos_2 = rx_from_RS422_2();


//  tx_to_esp();

  vTaskDelay(pdMS_TO_TICKS(1));

}

//======================================================
// void tx_to_esp() {
//   String msg_to_esp = "SRT";
//   msg_to_esp = msg_to_esp + ",";
//   msg_to_esp = msg_to_esp + String(RotaryEncoderPos1 * 0.06); 
//   msg_to_esp = msg_to_esp + ",";
//   msg_to_esp = msg_to_esp + String(AbsoluteEncoderPos1 * 360 / 1024);
//   msg_to_esp = msg_to_esp + ",";
//   msg_to_esp = msg_to_esp + String(data.buffer1);
//   msg_to_esp = msg_to_esp + ",";
//   msg_to_esp = msg_to_esp + String(data.buffer2);
//   msg_to_esp = msg_to_esp + ",";
//   msg_to_esp = msg_to_esp + String(data.buffer3);
//   msg_to_esp = msg_to_esp + ",";
//   msg_to_esp = msg_to_esp + String(RS422_Encoder_pos);
//   msg_to_esp = msg_to_esp + ",";
//   msg_to_esp = msg_to_esp + String(LoadCell_value1);
//   msg_to_esp = msg_to_esp + ",";
//   msg_to_esp = msg_to_esp + "END";
//   // Serial2.print(msg_to_esp);
//   Serial.println(msg_to_esp);
// }

float rx_from_RS422_1() {
  Serial1.write("3");
  vTaskDelay(pdMS_TO_TICKS(1));
  if(Serial1.available()>0){
    Serial1.readBytes((uint8_t*)&data1, sizeof(SensorData));

    float AbsoluteEncoderPos = (float)((data1.buffer1>>5)+(data1.buffer2<<11)+(data1.buffer3<<3))*360/524288;

   Serial.print("Recieved RS422 value : ");
   Serial.print(data1.buffer1, HEX);
   Serial.print("  ");
   Serial.print(data1.buffer2, HEX);
   Serial.print("  ");
   Serial.print(data1.buffer3, HEX);
   Serial.print("  ");
    Serial.println(AbsoluteEncoderPos);
    return AbsoluteEncoderPos;
  }
  else {
    // Serial.println("==== RS422 not Received ===");
    return 0;
  }
}

float rx_from_RS422_2() {
  Serial2.write("3");
  vTaskDelay(pdMS_TO_TICKS(1));
  if(Serial2.available()>0){
    Serial2.readBytes((uint8_t*)&data2, sizeof(SensorData));

    float AbsoluteEncoderPos = (float)((data2.buffer1>>5)+(data2.buffer2<<11)+(data2.buffer3<<3))*360/524288;

   Serial.print("Recieved RS422 value : ");
   Serial.print(data2.buffer1, HEX);
   Serial.print("  ");
   Serial.print(data2.buffer2, HEX);
   Serial.print("  ");
   Serial.print(data2.buffer3, HEX);
   Serial.print("  ");
    Serial.println(AbsoluteEncoderPos);
    return AbsoluteEncoderPos;
  }
  else {
    // Serial.println("==== RS422 not Received ===");
    return 0;
  }
}

void rx_from_loadcell() {
  LoadCell_value1 = scale1.get_units();
  // Serial.println(LoadCell_value1);
  vTaskDelay(pdMS_TO_TICKS(1));
}

//=============MULTITASK FUNCTIONS===============
void SENDCAN(void * parameter) {
  for(;;) {
    tx_CAN();
  }
}


void tx_CAN() {
  // Serial.println("Sensor CAN data send to ROS ");
  // ID for sensor == 2000(ascender) + 800 + (sensor type)
  // abs encoder    30
  // rot encoder    40
  // RS422 encoder  50
  // LoadCell       60
  // IMU            70
  // AbsEnc_CAN();
  // vTaskDelay(pdMS_TO_TICKS(2));
  // RotEnc_CAN();
  // vTaskDelay(pdMS_TO_TICKS(2));
  RS422Enc1_CAN();
  vTaskDelay(pdMS_TO_TICKS(2));
  RS422Enc2_CAN();
  vTaskDelay(pdMS_TO_TICKS(2));
  LoadCell_CAN();
  vTaskDelay(pdMS_TO_TICKS(3));
  if (twai_error_count > N_que) {
    Serial.println("twai error full -> restarting");
    // if (twai_initiate_recovery == ESP_OK) {
    //   twai_error_count = 0;
    //   Serial.println("CAN initiate recovery");
    // }
    twai_error_count = 0;
    ESP.restart();
  }
}
// Converting DEC to HEX functions
// void EBImu_CAN() {

//   int16_t euler_degree1_int16 = int16_t(euler[0]*100);
 
//   int16_t euler_degree2_int16 = int16_t(euler[1]*100);
 
//   int16_t euler_degree3_int16 = int16_t(euler[2]*100);
 
//   int16_t checksum_int16 = euler_degree1_int16+ euler_degree2_int16+ euler_degree3_int16;
 
//   SensorCan1.data[0] = byteRead(euler_degree1_int16, 1);
//   SensorCan1.data[1] = byteRead(euler_degree1_int16, 0);
//   SensorCan1.data[2] = byteRead(euler_degree2_int16, 1);
//   SensorCan1.data[3] = byteRead(euler_degree2_int16, 0);
//   SensorCan1.data[4] = byteRead(euler_degree3_int16, 1);
//   SensorCan1.data[5] = byteRead(euler_degree3_int16, 0);
//   SensorCan1.data[6] = byteRead(checksum_int16, 1);
//   SensorCan1.data[7] = byteRead(checksum_int16, 0);

//   // Serial.print("EBImu : ");
//   // Serial.println(" ");
//   // for (int i = 0; i <= 7; i++) {
//   //   Serial.print(SensorCan1.data[i], HEX);
//   //   Serial.print(" ");
//   // }
//   // Serial.println(" ");


//   if(twai_transmit(&SensorCan1, pdMS_TO_TICKS(10)) != ESP_OK) {
//     Serial.println("transmit error");
//     twai_error_count++;
//   }

// }

// void EBImu_Quaternion_CAN() {

//   int16_t quaternion_x_int16 = int16_t(quaternion[0]*100);
 
//   int16_t quaternion_y_int16 = int16_t(quaternion[1]*100);

//   int16_t quaternion_z_int16 = int16_t(quaternion[2]*100);
 
//   int16_t quaternion_w_int16 = int16_t(quaternion[3]*100);
 
//   SensorCan1.data[0] = byteRead(quaternion_x_int16, 1);
//   SensorCan1.data[1] = byteRead(quaternion_x_int16, 0);
//   SensorCan1.data[2] = byteRead(quaternion_y_int16, 1);
//   SensorCan1.data[3] = byteRead(quaternion_y_int16, 0);
//   SensorCan1.data[4] = byteRead(quaternion_z_int16, 1);
//   SensorCan1.data[5] = byteRead(quaternion_z_int16, 0);
//   SensorCan1.data[6] = byteRead(quaternion_w_int16, 1);
//   SensorCan1.data[7] = byteRead(quaternion_w_int16, 0);

//   // Serial.print("EBImu : ");
//   // Serial.println(" ");
//   // for (int i = 0; i <= 7; i++) {
//   //   Serial.print(SensorCan1.data[i], HEX);
//   //   Serial.print(" ");
//   // }
//   // Serial.println(" ");

//   if(twai_transmit(&SensorCan1, pdMS_TO_TICKS(10)) != ESP_OK) {
//     Serial.println("transmit error");
//     twai_error_count++;
//   }

// }

// void AbsEnc_CAN() {
  
//   // Absolute_Encoder_pos1 = 12345.67;   // HEX 00 12 D6 87
//   // Absolute_Encoder_pos2 = -12345.67;  // HEX FF ED 29 79  

//   AbsEnc_position1 = Absolute_Encoder_pos1 * 100;
//   AbsEnc_position1_int32 = int(AbsEnc_position1);

//   // AbsEnc_position2 = Absolute_Encoder_pos2 * 100;
//   // AbsEnc_position2_int32 = int(AbsEnc_position2);

//   SensorCan1.data[0] = byteRead(AbsEnc_position1_int32, 3);
//   SensorCan1.data[1] = byteRead(AbsEnc_position1_int32, 2);
//   SensorCan1.data[2] = byteRead(AbsEnc_position1_int32, 1);
//   SensorCan1.data[3] = byteRead(AbsEnc_position1_int32, 0);
//   // SensorCan1.data[4] = byteRead(AbsEnc_position2_int32, 3);
//   // SensorCan1.data[5] = byteRead(AbsEnc_position2_int32, 2);
//   // SensorCan1.data[6] = byteRead(AbsEnc_position2_int32, 1);
//   // SensorCan1.data[7] = byteRead(AbsEnc_position2_int32, 0);
//   SensorCan1.data[4] = 0x00;
//   SensorCan1.data[5] = 0x00;
//   SensorCan1.data[6] = 0x00;
//   SensorCan1.data[7] = 0x00;

//   // Serial.print("AbsEnc : ");
//   // Serial.println(" ");
//   // for (int i = 0; i <= 7; i++) {
//   //   Serial.print(SensorCan1.data[i], HEX);
//   //   Serial.print(" ");
//   // }
//   // Serial.println(" ");

//   twai_clear_transmit_queue();
//   if(twai_transmit(&SensorCan1, pdMS_TO_TICKS(100)) != ESP_OK) {
//     Serial.println("transmit error");
//     twai_error_count++;
//   }
//   else {
//     if(twai_error_count > 0){
//       Serial.println("transmit success");
//       twai_error_count--;
//     }
//   }

// }
// void RotEnc_CAN() {
  
//   // Rotary_Encoder_pos2 = 12345.67;   // HEX 00 12 D6 87
//   // Rotary_Encoder_pos1 = -12345.67;  // HEX FF ED 29 79  
  
//   RotEnc_position1 = Rotary_Encoder_pos1 * 100;
//   RotEnc_position1_int32 = int(RotEnc_position1);

//   // RotEnc_position2 = Rotary_Encoder_pos2 * 100;
//   // RotEnc_position2_int32 = int(RotEnc_position2);

//   SensorCan2.data[0] = byteRead(RotEnc_position1_int32, 3);
//   SensorCan2.data[1] = byteRead(RotEnc_position1_int32, 2);
//   SensorCan2.data[2] = byteRead(RotEnc_position1_int32, 1);
//   SensorCan2.data[3] = byteRead(RotEnc_position1_int32, 0);
//   // SensorCan2.data[4] = byteRead(RotEnc_position2_int32, 3);
//   // SensorCan2.data[5] = byteRead(RotEnc_position2_int32, 2);
//   // SensorCan2.data[6] = byteRead(RotEnc_position2_int32, 1);
//   // SensorCan2.data[7] = byteRead(RotEnc_position2_int32, 0);
//   SensorCan2.data[4] = 0x00;
//   SensorCan2.data[5] = 0x00;
//   SensorCan2.data[6] = 0x00;
//   SensorCan2.data[7] = 0x00;
  
//   // Serial.print("RotEnc : ");
//   // Serial.println(" ");
//   // for (int i = 0; i <= 7; i++) {
//   //   Serial.print(SensorCan2.data[i], HEX);
//   //   Serial.print(" ");
//   // }
//   // Serial.println(" ");
//   twai_clear_transmit_queue();
//   if(twai_transmit(&SensorCan2, pdMS_TO_TICKS(100)) != ESP_OK) {
//     Serial.println("transmit error");
//     twai_error_count++;
//   }
//   else {
//     if(twai_error_count > 0){
//       Serial.println("transmit success");
//       twai_error_count--;
//     }
//   }

// }
void RS422Enc1_CAN() {

  SensorCan1.data[0] = data1.buffer1;
  SensorCan1.data[1] = data1.buffer2;
  SensorCan1.data[2] = data1.buffer3;
  SensorCan1.data[3] = 0x00;
  SensorCan1.data[4] = 0x00;
  SensorCan1.data[5] = 0x00;
  SensorCan1.data[6] = 0x00;
  SensorCan1.data[7] = 0x00;

  // Serial.print("422Enc : ");
  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCan3.data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");
  twai_clear_transmit_queue();
  if(twai_transmit(&SensorCan1, pdMS_TO_TICKS(100)) != ESP_OK) {
    Serial.println("transmit error");
    twai_error_count++;
  }
  else {
    if(twai_error_count > 0){
      Serial.println("transmit success");
      twai_error_count--;
    }
  }

}

void RS422Enc2_CAN() {

  SensorCan2.data[0] = data2.buffer1;
  SensorCan2.data[1] = data2.buffer2;
  SensorCan2.data[2] = data2.buffer3;
  SensorCan2.data[3] = 0x00;
  SensorCan2.data[4] = 0x00;
  SensorCan2.data[5] = 0x00;
  SensorCan2.data[6] = 0x00;
  SensorCan2.data[7] = 0x00;

  // Serial.print("422Enc : ");
  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCan3.data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");
  twai_clear_transmit_queue();
  if(twai_transmit(&SensorCan2, pdMS_TO_TICKS(100)) != ESP_OK) {
    Serial.println("transmit error");
    twai_error_count++;
  }
  else {
    if(twai_error_count > 0){
      Serial.println("transmit success");
      twai_error_count--;
    }
  }

}

void LoadCell_CAN() {
  // LoadCell_value1 = 12345.67;   // HEX 00 12 D6 87
  // LoadCell_value2 = -12345.67;  // HEX FF ED 29 79  

  loadcell_degree1 = LoadCell_value1 * 100;
  loadcell_degree1_int32 = int(loadcell_degree1);

  // loadcell_degree2 = LoadCell_value2 * 100;
  // loadcell_degree2_int32 = int(loadcell_degree2);

  SensorCan4.data[0] = byteRead(loadcell_degree1_int32, 3);
  SensorCan4.data[1] = byteRead(loadcell_degree1_int32, 2);
  SensorCan4.data[2] = byteRead(loadcell_degree1_int32, 1);
  SensorCan4.data[3] = byteRead(loadcell_degree1_int32, 0);
  // SensorCan4.data[4] = byteRead(loadcell_degree2_int32, 3);
  // SensorCan4.data[5] = byteRead(loadcell_degree2_int32, 2);
  // SensorCan4.data[6] = byteRead(loadcell_degree2_int32, 1);
  // SensorCan4.data[7] = byteRead(loadcell_degree2_int32, 0);
  SensorCan4.data[4] = 0x00;
  SensorCan4.data[5] = 0x00;
  SensorCan4.data[6] = 0x00;
  SensorCan4.data[7] = 0x00;

  // Serial.print("LoadCell : ");
  // Serial.println(" ");
  // Serial.print("LoadCell : ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCan4.data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");
  twai_clear_transmit_queue();
  if(twai_transmit(&SensorCan4, pdMS_TO_TICKS(100)) != ESP_OK) {
    Serial.println("transmit error");
    twai_error_count++;
  }
  else {
    if(twai_error_count > 0){
      Serial.println("transmit success");
      twai_error_count--;
    }
  }

}
