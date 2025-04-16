#include <Arduino.h>
#include "driver/twai.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#define byteRead(x,n) ( * ( (unsigned char*)(&x) + n) )

// [RF]
int PIN_THRO = 4; // controller 3
int PIN_AILE = 5; // conttoller 4
int PIN_ELEV = 6; // controller 1
int PIN_RUDD = 7; // controller 2 
int PIN_GEAR = 15; // gear
int PIN_AUX1 = 16; // aux1

float MAX_GEAR = 2000;
float UP_GEAR = 1700;
float DOWN_GEAR = 1300;
float MIN_GEAR = 1000;
float MAX_THRO = 1950;
float UP_THRO = 1800;
float DOWN_THRO = 1200;
float MIN_THRO = 1050;
float MAX_AILE = 1897;
float UP_AILE = 1700;
float DOWN_AILE = 1300;
float MIN_AILE = 1098;
float MAX_ELEV = 1900;
float UP_ELEV = 1700;
float DOWN_ELEV = 1200;
float MIN_ELEV = 1050;
float MAX_RUDD = 1950;
float UP_RUDD = 1700;
float DOWN_RUDD = 1250;
float MIN_RUDD = 1050;

unsigned long THRO = 0.0; 
unsigned long AILE = 0.0; 
unsigned long ELEV = 0.0;
unsigned long RUDD = 0.0;
unsigned long GEAR = 0.0;
unsigned long AUX1 = 0.0;

int Thro_1 = 0;
int Aile_1 = 0;
int Elev_1 = 0;
int Rudd_1 = 0;

// [TWAI]
// CAN tx: 47 rx: 21
twai_message_t MotorCan1, MotorCan2, SensorCan1, SensorCan2;
uint32_t twai_error_count = 0; 
#define N_que 30

float AbsEnc_position1 = 0.0;
int AbsEnc_position1_int32 = 0;
float RotEnc_position1 = 0.0;
int RotEnc_position1_int32 = 0;
float loadcell_degree1 = 0.0;
int loadcell_degree1_int32 = 0;

// [Serial1] EBIMU
//define what pin rx2 tx2 is going to be
#define rx1 42                                          
#define tx1 2
unsigned long timeout = 50; //50ms

// // [Serial2] ESP
// //define what pin rx2 tx2 is going to be
// #define rx2 47                                          
// #define tx2 21
// unsigned long timeout = 50; //50ms

// [EBIMU]
// parameter for EMimu
float euler[3];
double quaternion[4];
double acceleration[3];

// [Motor]
float motor_position[2];
float rpm[2];

// [Functions]
void tx_to_esp();
void rx_from_imu();
void rx_from_imu_quaternion();
void RF_recieve_values();
void Motor_control();
void tx_CAN();
void EMImu_CAN();
void EBImu_Quaternion_CAN();
void EBImu_Acceleration_CAN();
void Motor_Control_Velocity_CAN1(uint32_t ID, int rpm);
void Motor_Control_Velocity_CAN2(uint32_t ID, int rpm);

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
  Serial1.begin(115200, SERIAL_8N1, rx1, tx1);
  // Serial2.begin(115200, SERIAL_8N1, rx2, tx2);
  Serial1.setTimeout(timeout);
  // Serial2.setTimeout(timeout);

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

  // RF receiver
  // ESP32 는 pinmode 인풋지정해줘야한다. 아두이노는 괜춘
  pinMode(PIN_THRO, INPUT); // Thro
  pinMode(PIN_AILE, INPUT); // AILE
  pinMode(PIN_ELEV, INPUT); // ELEV
  pinMode(PIN_RUDD, INPUT); // Rudd
  pinMode(PIN_GEAR, INPUT); // Gear
  pinMode(PIN_AUX1, INPUT); // Aux1der1PinA), updateRotaryEncoder1, CHANGE);
 
  // CAN setting
  // MotorCan1.identifier = 0x321;
  // MotorCan2.identifier = 0x322;
  // MotorCan1.extd = 0;
  // MotorCan2.extd = 0;
  // MotorCan1.data_length_code = 8;
  // MotorCan2.data_length_code = 8;
  SensorCan1.identifier = 0x2871;
  SensorCan1.extd = 1;
  SensorCan1.data_length_code = 8;
  SensorCan2.identifier = 0x2881;
  SensorCan2.extd = 1;
  SensorCan2.data_length_code = 8;

}

void loop() {

  rx_from_imu_quaternion();
  vTaskDelay(pdMS_TO_TICKS(1));

//  RF_recieve_values();
  vTaskDelay(pdMS_TO_TICKS(1));

  Motor_control();

}
//======================================================


void rx_from_imu() {
  if (Serial1.available()>0) {
      String msg_from_imu = Serial1.readStringUntil(0x0a);
      Serial.println(msg_from_imu);
    if (msg_from_imu.startsWith("*")) {
      int term1 = msg_from_imu.indexOf("*");               // *
      int term2 = msg_from_imu.indexOf(",", term1 + 1);    // pitch
      int term3 = msg_from_imu.indexOf(",", term2 + 1);    // roll
      int term4 = msg_from_imu.indexOf(",", term3 + 1);    // yaw
  
      String pitch_string = msg_from_imu.substring(term1 + 1, term2);
      String roll_string = msg_from_imu.substring(term2 + 1, term3);
      String yaw_string = msg_from_imu.substring(term3 + 1, term4);
  
      euler[0] = pitch_string.toFloat();
      euler[1] = roll_string.toFloat();
      euler[2] = yaw_string.toFloat();
  
      Serial.print("Pitch :"); Serial.print(euler[0]);   Serial.print(" ");
      Serial.print("Roll :"); Serial.print(euler[1]);   Serial.print(" ");
      Serial.print("Yaw :"); Serial.print(euler[2]);   Serial.println(" ");
    }
    else {
      Serial.println("====== not imu data ======");
    }
  }
  else {
    Serial.println("==== No data Received from imu ====");
  }

}

void rx_from_imu_quaternion() {
  if (Serial1.available()>0) {
      String msg_from_imu = Serial1.readStringUntil(0x0a);
//      Serial.println(msg_from_imu);
    if (msg_from_imu.startsWith("*")) {
      int term1 = msg_from_imu.indexOf("*");               // *
      int term2 = msg_from_imu.indexOf(",", term1 + 1);    // z
      int term3 = msg_from_imu.indexOf(",", term2 + 1);    // y
      int term4 = msg_from_imu.indexOf(",", term3 + 1);    // x
      int term5 = msg_from_imu.indexOf(",", term4 + 1);    // w
      int term6 = msg_from_imu.indexOf(",", term5 + 1);    // accx
      int term7 = msg_from_imu.indexOf(",", term6 + 1);    // accy
      int term8 = msg_from_imu.indexOf(",", term7 + 1);    // accz
  
      String z_string = msg_from_imu.substring(term1 + 1, term2);
      String y_string = msg_from_imu.substring(term2 + 1, term3);
      String x_string = msg_from_imu.substring(term3 + 1, term4);
      String w_string = msg_from_imu.substring(term4 + 1, term5);
      String accx_string = msg_from_imu.substring(term5 + 1, term6);
      String accy_string = msg_from_imu.substring(term6 + 1, term7);
      String accz_string = msg_from_imu.substring(term7 + 1, term8);

      
      if (!(-1<=x_string.toDouble() && x_string.toDouble() <= 1)){
      //  Serial.println("no");
        return;
      }
      if (!(-1<=y_string.toDouble() && y_string.toDouble() <= 1)){
      //  Serial.println("no");
        return;
      }
      if (!(-1<=z_string.toDouble() && z_string.toDouble() <= 1)){
      //  Serial.println("no");
        return;
      }
      if (!(-1<=w_string.toDouble() && w_string.toDouble() <= 1)){
      //  Serial.println("no");
        return;
      }
 
      quaternion[0] = x_string.toDouble();
      quaternion[1] = y_string.toDouble();
      quaternion[2] = z_string.toDouble();
      quaternion[3] = w_string.toDouble();

      acceleration[0] = accx_string.toDouble();
      acceleration[1] = accy_string.toDouble();
      acceleration[2] = accz_string.toDouble();
  
      Serial.print("x :"); Serial.print(quaternion[0],4);   Serial.print(" ");
      Serial.print("y :"); Serial.print(quaternion[1],4);   Serial.print(" ");
      Serial.print("z :"); Serial.print(quaternion[2],4);   Serial.print(" ");
      Serial.print("w :"); Serial.print(quaternion[3],4);   Serial.print(" ");
      Serial.print("accx :"); Serial.print(acceleration[0],4);   Serial.print(" ");
      Serial.print("accy :"); Serial.print(acceleration[1],4);   Serial.print(" ");
      Serial.print("accz :"); Serial.print(acceleration[2],4);   Serial.println(" ");
      
    }
    else {
    //  Serial.println("====== not imu data ======");
    }
  }
  else {
  //  Serial.println("==== No data Received from imu ====");
  }

}


void RF_recieve_values() { //44ms
  ELEV = 0;
  RUDD = 0;
  THRO = 0;
  AILE = 0;
  THRO = pulseIn(PIN_THRO, HIGH, 50000); //Elev 펄스를 읽습니다
  // Thro_1 = map(THRO, MIN_THRO, MAX_THRO, -255, 255); //읽힌 펄스의 값을 PWM범위로 다시 바꾸어줍니다.  1900 1102

  AILE = pulseIn(PIN_AILE, HIGH, 50000); //Elev 펄스를 읽습니다
  // Aile_1 = map(AILE, MIN_AILE, MAX_AILE, -255, 255); //읽힌 펄스의 값을 PWM범위로 다시 바꾸어줍니다. 1900 1105
  
  ELEV = pulseIn(PIN_ELEV, HIGH, 50000); //Elev 펄스를 읽습니다
  // Elev_1 = map(ELEV, MIN_ELEV, MAX_ELEV, -255, 255); //읽힌 펄스의 값을 PWM범위로 다시 바꾸어줍니다. 1891 1116
  
  RUDD = pulseIn(PIN_RUDD, HIGH, 50000); //Elev 펄스를 읽습니다
  // Rudd_1 = map(RUDD, MIN_RUDD, MAX_RUDD, -255, 255); //읽힌 펄스의 값을 PWM범위로 다시 바꾸어줍니다. 1895 1105
  
  GEAR = pulseIn(PIN_GEAR, HIGH, 50000); 
  // AUX1 = pulseIn(PIN_AUX1, HIGH, 10000); 

  Serial.print(THRO);
  Serial.print(" ");
  Serial.print(AILE);
  Serial.print(" ");
  Serial.print(ELEV);
  Serial.print(" ");
  Serial.print(RUDD);
  Serial.print(" ");
  Serial.println(GEAR);
  // Serial.print(" ");
  // Serial.println(AUX1);
    
  // Serial.print(Thro_1);
  // Serial.print(" ");
  // Serial.print(Aile_1);
  // Serial.print(" ");
  // Serial.print(Elev_1);
  // Serial.print(" ");
  // Serial.println(Rudd_1);
}

void Motor_control() {
  //  RF off - ROS2 system
  if (!(GEAR > MIN_GEAR) && (GEAR < MAX_GEAR)) {
    // Serial.println("ROS mode");

  }
  // RF mode
  else {
    // Serial.println("RF mode");

    // GEAR 0 : velocity mode 2
    if (GEAR > UP_GEAR && GEAR < MAX_GEAR) {
      // Serial.println("Gear 0");

      if (ELEV > UP_ELEV && ELEV < MAX_ELEV) {
        // 1 up
        rpm[0] = 10;
        
      }
      else if (ELEV < DOWN_ELEV && ELEV > MIN_ELEV){
        // 1 down
        rpm[0] = -10;

      }
      else {
        // 1 stop
        rpm[0] = 0;
        
      }

      if (THRO > UP_THRO && THRO < MAX_THRO) {
        // 1 up
        rpm[1] = 10;
        
      }
      else if (THRO < DOWN_THRO && THRO > MIN_THRO){
        // 1 down
        rpm[1] = -10;

      }
      else {
        // rpm1 = 0;
        rpm[1] = 0;
        
      }
      
    }
    // GEAR 2 : stop mode
    else if (GEAR < DOWN_GEAR && GEAR > MIN_GEAR) {
      // Serial.println("Gear 2");

    }

    // GEAR 1 : velocity mode
    else {
      // Serial.println("Gear 1");

      if (ELEV > UP_ELEV && ELEV < MAX_ELEV) {
        // up
        rpm[0] = 10;
        rpm[1] = 10;
        
      }
      else if (ELEV < DOWN_ELEV && ELEV > MIN_ELEV){
        // down
        rpm[0] = -10;
        rpm[1] = -10;

      }
      else {
        if (RUDD > UP_RUDD && RUDD < MAX_RUDD) {
        // up
          rpm[0] = 10;
          rpm[1] = -10;
        }
        else if (RUDD < DOWN_RUDD && RUDD > MIN_RUDD){
        // down
          rpm[0] = -10;
          rpm[1] = 10;
        }
        else {
        // stop
      
          rpm[0] = 0;
          rpm[1] = 0;
        }
      }
    }
  }
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
  EBImu_Quaternion_CAN();
  vTaskDelay(pdMS_TO_TICKS(4));

  EBImu_Acceleration_CAN();
  vTaskDelay(pdMS_TO_TICKS(4));

  //  if (GEAR > MIN_GEAR && GEAR < MAX_GEAR) {
//
//    if (GEAR > UP_GEAR && GEAR < MAX_GEAR) {
//      // Serial.println("Gear 0");
//  
//      Motor_Control_Velocity_CAN1(301, rpm[0]);
//      vTaskDelay(pdMS_TO_TICKS(2));
//      Motor_Control_Velocity_CAN2(302, rpm[1]);
//      vTaskDelay(pdMS_TO_TICKS(2));
//  
//        
//      }
//      // GEAR 2 : stop mode
//    else if (GEAR < DOWN_GEAR && GEAR > MIN_GEAR) {
//    }
//  
//      // GEAR 1 : velocity mode
//    else {
//      Motor_Control_Velocity_CAN1(301, rpm[0]);
//      vTaskDelay(pdMS_TO_TICKS(2));
//      Motor_Control_Velocity_CAN2(302, rpm[1]);
//      vTaskDelay(pdMS_TO_TICKS(2));
//    }
//  }
//  else {
//    vTaskDelay(pdMS_TO_TICKS(4));
//  }
  
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
void EBImu_CAN() {

  int16_t euler_degree1_int16 = int16_t(euler[0]*100);
 
  int16_t euler_degree2_int16 = int16_t(euler[1]*100);
 
  int16_t euler_degree3_int16 = int16_t(euler[2]*100);
 
  int16_t checksum_int16 = euler_degree1_int16+ euler_degree2_int16+ euler_degree3_int16;
 
  SensorCan1.data[0] = byteRead(euler_degree1_int16, 1);
  SensorCan1.data[1] = byteRead(euler_degree1_int16, 0);
  SensorCan1.data[2] = byteRead(euler_degree2_int16, 1);
  SensorCan1.data[3] = byteRead(euler_degree2_int16, 0);
  SensorCan1.data[4] = byteRead(euler_degree3_int16, 1);
  SensorCan1.data[5] = byteRead(euler_degree3_int16, 0);
  SensorCan1.data[6] = byteRead(checksum_int16, 1);
  SensorCan1.data[7] = byteRead(checksum_int16, 0);

  // Serial.print("EBImu : ");
  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCan1.data[i], HEX);
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

void EBImu_Quaternion_CAN() {

  int16_t quaternion_x_int16 = int16_t(quaternion[0]*10000);
 
  int16_t quaternion_y_int16 = int16_t(quaternion[1]*10000);

  int16_t quaternion_z_int16 = int16_t(quaternion[2]*10000);
 
  int16_t quaternion_w_int16 = int16_t(quaternion[3]*10000);
 
  SensorCan1.data[0] = byteRead(quaternion_x_int16, 1);
  SensorCan1.data[1] = byteRead(quaternion_x_int16, 0);
  SensorCan1.data[2] = byteRead(quaternion_y_int16, 1);
  SensorCan1.data[3] = byteRead(quaternion_y_int16, 0);
  SensorCan1.data[4] = byteRead(quaternion_z_int16, 1);
  SensorCan1.data[5] = byteRead(quaternion_z_int16, 0);
  SensorCan1.data[6] = byteRead(quaternion_w_int16, 1);
  SensorCan1.data[7] = byteRead(quaternion_w_int16, 0);

  // Serial.print("EBImu : ");
  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCan1.data[i], HEX);
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



void EBImu_Acceleration_CAN() {

  int16_t acc_x_int16 = int16_t(acceleration[0]*1000);
 
  int16_t acc_y_int16 = int16_t(acceleration[1]*1000);

  int16_t acc_z_int16 = int16_t(acceleration[2]*1000);

  int16_t checksum_int16 = acc_x_int16+ acc_y_int16+ acc_z_int16;
 
  SensorCan2.data[0] = byteRead(acc_x_int16, 1);
  SensorCan2.data[1] = byteRead(acc_x_int16, 0);
  SensorCan2.data[2] = byteRead(acc_y_int16, 1);
  SensorCan2.data[3] = byteRead(acc_y_int16, 0);
  SensorCan2.data[4] = byteRead(acc_z_int16, 1);
  SensorCan2.data[5] = byteRead(acc_z_int16, 0);
  SensorCan2.data[6] = byteRead(checksum_int16, 1);
  SensorCan2.data[7] = byteRead(checksum_int16, 0);

  // Serial.print("acc : ");
  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCan2.data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");

  if(twai_transmit(&SensorCan2, pdMS_TO_TICKS(10)) != ESP_OK) {
    Serial.println("transmit error");
    twai_error_count++;
  }

}

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


//   if(twai_transmit(&SensorCan1, pdMS_TO_TICKS(10)) != ESP_OK) {
//     Serial.println("transmit error");
//     twai_error_count++;
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

//   if(twai_transmit(&SensorCan2, pdMS_TO_TICKS(10)) != ESP_OK) {
//     Serial.println("transmit error");
//     twai_error_count++;
//   }

// }
// void RS422Enc_CAN() {

//   SensorCan3.data[0] = data.buffer1;
//   SensorCan3.data[1] = data.buffer2;
//   SensorCan3.data[2] = data.buffer3;
//   SensorCan3.data[3] = 0x00;
//   SensorCan3.data[4] = 0x00;
//   SensorCan3.data[5] = 0x00;
//   SensorCan3.data[6] = 0x00;
//   SensorCan3.data[7] = 0x00;

//   // Serial.print("422Enc : ");
//   // Serial.println(" ");
//   // for (int i = 0; i <= 7; i++) {
//   //   Serial.print(SensorCan3.data[i], HEX);
//   //   Serial.print(" ");
//   // }
//   // Serial.println(" ");

//   if(twai_transmit(&SensorCan3, pdMS_TO_TICKS(10)) != ESP_OK) {
//     Serial.println("transmit error");
//     twai_error_count++;
//   }

// }
// void LoadCell_CAN() {
//   // LoadCell_value1 = 12345.67;   // HEX 00 12 D6 87
//   // LoadCell_value2 = -12345.67;  // HEX FF ED 29 79  

//   loadcell_degree1 = LoadCell_value1 * 100;
//   loadcell_degree1_int32 = int(loadcell_degree1);

//   // loadcell_degree2 = LoadCell_value2 * 100;
//   // loadcell_degree2_int32 = int(loadcell_degree2);

//   SensorCan4.data[0] = byteRead(loadcell_degree1_int32, 3);
//   SensorCan4.data[1] = byteRead(loadcell_degree1_int32, 2);
//   SensorCan4.data[2] = byteRead(loadcell_degree1_int32, 1);
//   SensorCan4.data[3] = byteRead(loadcell_degree1_int32, 0);
//   // SensorCan4.data[4] = byteRead(loadcell_degree2_int32, 3);
//   // SensorCan4.data[5] = byteRead(loadcell_degree2_int32, 2);
//   // SensorCan4.data[6] = byteRead(loadcell_degree2_int32, 1);
//   // SensorCan4.data[7] = byteRead(loadcell_degree2_int32, 0);
//   SensorCan4.data[4] = 0x00;
//   SensorCan4.data[5] = 0x00;
//   SensorCan4.data[6] = 0x00;
//   SensorCan4.data[7] = 0x00;

//   // Serial.print("LoadCell : ");
//   // Serial.println(" ");
//   // Serial.print("LoadCell : ");
//   // for (int i = 0; i <= 7; i++) {
//   //   Serial.print(SensorCan4.data[i], HEX);
//   //   Serial.print(" ");
//   // }
//   // Serial.println(" ");

//   if(twai_transmit(&SensorCan4, pdMS_TO_TICKS(10)) != ESP_OK) {
//     Serial.println("transmit error");
//     twai_error_count++;
//   }

// }



void Motor_Control_Velocity_CAN1(uint32_t ID, int rpm) { 

  //float motor_position_degree = 0.0;
  float motor_velocity = - rpm * 64*21;
  int motor_velocity_int32 = int(motor_velocity);

  MotorCan1.data[0] = byteRead(motor_velocity_int32,3);
  MotorCan1.data[1] = byteRead(motor_velocity_int32,2);
  MotorCan1.data[2] = byteRead(motor_velocity_int32,1);
  MotorCan1.data[3] = byteRead(motor_velocity_int32,0);
  MotorCan1.data[4] = byte(0);
  MotorCan1.data[5] = byte(0);
  MotorCan1.data[6] = byte(0);
  MotorCan1.data[7] = byte(0);

  // CAN_send(MotorCan1);
  if(twai_transmit(&MotorCan1, pdMS_TO_TICKS(10)) != ESP_OK) {
    Serial.println("transmit error");
    twai_error_count++;
  }
}

void Motor_Control_Velocity_CAN2(uint32_t ID, int rpm) { 

  //float motor_position_degree = 0.0;
  float motor_velocity = rpm * 64*21;
  int motor_velocity_int32 = int(motor_velocity);

  MotorCan2.data[0] = byteRead(motor_velocity_int32,3);
  MotorCan2.data[1] = byteRead(motor_velocity_int32,2);
  MotorCan2.data[2] = byteRead(motor_velocity_int32,1);
  MotorCan2.data[3] = byteRead(motor_velocity_int32,0);
  MotorCan2.data[4] = byte(0);
  MotorCan2.data[5] = byte(0);
  MotorCan2.data[6] = byte(0);
  MotorCan2.data[7] = byte(0);

  // CAN_send(MotorCan2);
  if(twai_transmit(&MotorCan2, pdMS_TO_TICKS(10)) != ESP_OK) {
    Serial.println("transmit error");
    twai_error_count++;
  }
}
