#include <CAN.h>

#define byteRead(x,n) ( * ( (unsigned char*)(&x) + n) )

// [CAN]
// define CAN struct
struct CanData {
  uint32_t Id;
  int length;
  int Extened;
  int RTR;
  byte Data[8];
};
// pin select
int rx = 18;
int tx = 19;
// parameter for CAN
byte candata[8];
CanData SensorCan1, SensorCan2, SensorCan3, SensorCan4;

// [Serial1]
//define what pin rx2 tx2 is going to be
// #define rx1 16                                          
// #define tx1 17

// [Serial2]
//define what pin rx2 tx2 is going to be
#define rx2 16                                          
#define tx2 17
unsigned long timeout = 50; //50ms


// [Encoder]
float Rotary_Encoder_pos1 = 0.0;
float Rotary_Encoder_pos2 = 0.0;
float Absolute_Encoder_pos1 = 0.0;
float Absolute_Encoder_pos2 = 0.0;

// [LoadCell]
// #define calibration_factor1 -2180
// #define dtout1 14  //DT 전선 연결
// #define clk1 27    // SCLK 전선 연결
// #define calibration_factor2 -2180
// #define dtout2 26  //DT 전선 연결
// #define clk2 25    // SCLK 전선 연결
// HX711 scale1;
// HX711 scale2;
float LoadCell_value1 = 0.0;
float LoadCell_value2 = 0.0;

// [RS422 abs Encoder]
struct SensorData {
  byte buffer1;
  byte buffer2;
  byte buffer3;
}data;
// SensorData data1, data2;
float RS422_Encoder_pos1 = 0.0;
float RS422_Encoder_pos2 = 0.0;

void rx_from_esp32();
void tx_CAN_to_ROS();
void tx_CAN();
void AbsEnc_CAN();
void RotEnc_CAN();
void RS422Enc_CAN();
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
  // Serial.setTimeout(timeout);
  Serial2.begin(115200, SERIAL_8N1, rx2, tx2);
  Serial2.setTimeout(timeout);

  CAN.setPins(rx, tx);
  CAN.setTimeout(timeout);

  if (!CAN.begin(1000E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  // RF receiver
  // ESP32 는 pinmode 인풋지정해줘야한다. 아두이노는 괜춘
  // pinMode(PIN_THRO, INPUT_PULLUP); // Thro
  // pinMode(PIN_AILE, INPUT_PULLUP); // AILE
  // pinMode(PIN_ELEV, INPUT_PULLUP); // ELEV
  // pinMode(PIN_RUDD, INPUT_PULLUP); // Rudd
  // pinMode(PIN_GEAR, INPUT); // Gear
  // pinMode(PIN_AUX1, INPUT); // Aux1

  SensorCan1.Id = 0x2832; //ABS
  SensorCan2.Id = 0x2842; //ROT
  SensorCan3.Id = 0x2852; //RS
  SensorCan4.Id = 0x2862; //LOAD
  SensorCan1.length = 8;
  SensorCan2.length = 8;
  SensorCan3.length = 8;
  SensorCan4.length = 8;  

}
void loop() {
  
  rx_from_esp32(); //0ms
  vTaskDelay(pdMS_TO_TICKS(1));

}

//====================================================================================
// Functions

// rx serial from esp32s3
void rx_from_esp32() { 
  if (Serial2.available()>0) {
      String msg_from_esp = Serial2.readStringUntil('D');
//      Serial.print("Received Data is : ");
//      Serial.println(msg_from_esp);
    if (msg_from_esp.startsWith("SRT")) {
      int term1 = msg_from_esp.indexOf(",");               // SRT
      int term2 = msg_from_esp.indexOf(",", term1 + 1);    // Rotary_Encoder_pos1
      int term3 = msg_from_esp.indexOf(",", term2 + 1);    // Absolute_Encoder_pos1
      int term4 = msg_from_esp.indexOf(",", term3 + 1);    // RS422_buffer1
      int term5 = msg_from_esp.indexOf(",", term4 + 1);    // RS422_buffer2
      int term6 = msg_from_esp.indexOf(",", term5 + 1);    // RS422_buffer3
      int term7 = msg_from_esp.indexOf(",", term6 + 1);    // Loadcell_value1
      
      String Rotary_Encoder_pos1_string = msg_from_esp.substring(term1 + 1, term2);
      String Absolute_Encoder_pos1_string = msg_from_esp.substring(term2 + 1, term3);
      String RS422_Encoder_buffer1_string = msg_from_esp.substring(term3 + 1, term4);
      String RS422_Encoder_buffer2_string = msg_from_esp.substring(term4 + 1, term5);
      String RS422_Encoder_buffer3_string = msg_from_esp.substring(term5 + 1, term6);
      String LoadCell_value1_string = msg_from_esp.substring(term6 + 1, term7);
      String END_string = msg_from_esp.substring(term7 + 1, term7 + 3);

      if (END_string != "EN") {
//        Serial.println(END_string);
        return;
      }
      
      Rotary_Encoder_pos1 = Rotary_Encoder_pos1_string.toFloat();
      // Rotary_Encoder_pos2 = Rotary_Encoder_pos2_string.toFloat();
      Absolute_Encoder_pos1 = Absolute_Encoder_pos1_string.toFloat();
      // Absolute_Encoder_pos2 = Absolute_Encoder_pos2_string.toFloat();
      data.buffer1 = uint8_t(RS422_Encoder_buffer1_string.toInt());
      data.buffer2 = uint8_t(RS422_Encoder_buffer2_string.toInt());
      data.buffer3 = uint8_t(RS422_Encoder_buffer3_string.toInt());
      LoadCell_value1 = LoadCell_value1_string.toFloat();
      // LoadCell_value2 = LoadCell2_value_string.toFloat();
      RS422_Encoder_pos1 = (float)((data.buffer3>>5)+(data.buffer1<<11)+(data.buffer2<<3))*360/524288;


//      Serial.print(Rotary_Encoder_pos1);
//      Serial.print(" , ");
//      // Serial.print(Rotary_Encoder_pos2);
//      // Serial.print(" , ");
//      Serial.print(Absolute_Encoder_pos1);
//      Serial.print(" , ");
//      // Serial.println(Absolute_Encoder_pos2);
//      Serial.print(RS422_Encoder_pos1);
//      Serial.print(" , ");
//      Serial.println(LoadCell_value1);
      

    }
    else {
      // Serial.println("====== not esp data ======");
    }
  }
  else {
    // Serial.println("==== No data Received from esp32 ====");
  }

}

CanData CAN_receive(uint32_t MaskId){
  CanData ReceiveCanData;
  int packetSize = CAN.parsePacket();
  if (packetSize){ // is there CAN data received?
    uint32_t Id_identifier = CAN.packetId();

    // Serial.print(Id_identifier); Serial.print(" ? ");
    // Serial.println(MaskId);
    // Serial.println(Id_identifier == MaskId);

    if (Id_identifier == MaskId) { // has CAN Data right ID? 
      // Serial.print("Received ");
      ReceiveCanData.length = packetSize;
      ReceiveCanData.Id = Id_identifier;
    
      if(CAN.packetExtended()){
        // ReceiveCanData.Extened = 1;
        // Serial.print("extended ");
      }
      else{
        ReceiveCanData.Extened = 0;
      }

      if (CAN.packetRtr()) {
        // ReceiveCanData.RTR = 1;
        // Serial.print("RTR ");
      }
      else{
        ReceiveCanData.RTR = 0;
      }
      // Serial.print("packet with id 0x");
      // Serial.print(CAN.packetId(), HEX);
      // Serial.print("  ");

      int i = 0;

      while (CAN.available()) {
    
        //Serial.println(CAN.read(), HEX);
        ReceiveCanData.Data[i] = CAN.read();
        i = i+1;
      }

      // Serial.print(ReceiveCanData.Data[0], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[1], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[2], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[3], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[4], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[5], HEX);
      // Serial.print("  ");
      // Serial.print(ReceiveCanData.Data[6], HEX);
      // Serial.print("  ");
      // Serial.println(ReceiveCanData.Data[7], HEX);

    }
    else {
      // Serial.println("=== Wrong ID ===="); 
      //ReceiveCanData.Extened = 0;
      //ReceiveCanData.RTR = 1;
      //for(int i = 0; i < ReceiveCanData.length ; i++) {
      //  ReceiveCanData.Data[i] = byte(0);
      //}
    }
  }
  else {
    // Serial.println("=== No CAN Received ====");
  }
  return ReceiveCanData;
}

void CAN_send(CanData InputCanData){
  CAN.beginPacket(InputCanData.Id);
  CAN.write(InputCanData.Data, InputCanData.length);
  CAN.endPacket();
}

void CAN_EXTsend(CanData InputCanData){
  CAN.beginExtendedPacket(InputCanData.Id);
  CAN.write(InputCanData.Data, InputCanData.length);
  CAN.endPacket();
}

float rx_from_RS422() {
  Serial2.write("3");
  delay(1);
  if(Serial2.available()){
    Serial2.readBytes((uint8_t*)&data, sizeof(SensorData));

    float AbsoluteEncoderPos = (float)((data.buffer1>>5)+(data.buffer2<<11)+(data.buffer3<<3))*360/524288;

    // Serial.print("Recieved RS422 value : ");
    // Serial.print(data.buffer1, HEX);
    // Serial.print("  ");
    // Serial.print(data.buffer2, HEX);
    // Serial.print("  ");
    // Serial.print(data.buffer3, HEX);
    // Serial.print("  ");
     Serial.println(AbsoluteEncoderPos);
    return AbsoluteEncoderPos;
  }
  else {
    // Serial.println("==== RS422 not Received ===");
    return 0;
  }
}

void SENDCAN(void * parameter) {
  for(;;) {
    tx_CAN();
    // vTaskDelay(0.001);
  }
}

float AbsEnc_position1 = 0.0;
int AbsEnc_position1_int32 = 0;
float AbsEnc_position2 = 0.0;
int AbsEnc_position2_int32 = 0;
float RotEnc_position1 = 0.0;
int RotEnc_position1_int32 = 0;
float RotEnc_position2 = 0.0;
int RotEnc_position2_int32 = 0;
float loadcell_degree1 = 0.0;
int loadcell_degree1_int32 = 0;
float loadcell_degree2 = 0.0;
int loadcell_degree2_int32 = 0;
 
void tx_CAN() {
  // Serial.println("Sensor CAN data send to ROS ");
  AbsEnc_CAN();
  RotEnc_CAN();
  RS422Enc_CAN();
  LoadCell_CAN();

}
// Converting DEC to HEX functions
void AbsEnc_CAN() {
  // Serial.print("AbsEnc : ");
  // CanData SensorCanData; //Absolute_Encoder_pos
  // SensorCanData.Id = 0x2381;
  // SensorCanData.length = 8;
  
  // Absolute_Encoder_pos1 = 12345.67;   // HEX 00 12 D6 87
  // Absolute_Encoder_pos2 = -12345.67;  // HEX FF ED 29 79  

  AbsEnc_position1 = Absolute_Encoder_pos1 * 100;
  AbsEnc_position1_int32 = int(AbsEnc_position1);
  // int AbsEnc_position1_int32_HIGH = AbsEnc_position1_int32 >> 16;
  // int AbsEnc_position1_int32_LOW = AbsEnc_position1_int32-(AbsEnc_position1_int32_HIGH<<16);
  // int AbsEnc_position1_int32_HIGH_HIGH = AbsEnc_position1_int32_HIGH >> 8;
  // int AbsEnc_position1_int32_HIGH_LOW = AbsEnc_position1_int32_HIGH-(AbsEnc_position1_int32_HIGH_HIGH<<8);
  // int AbsEnc_position1_int32_LOW_HIGH = AbsEnc_position1_int32_LOW >> 8;
  // int AbsEnc_position1_int32_LOW_LOW = AbsEnc_position1_int32_LOW-(AbsEnc_position1_int32_LOW_HIGH<<8);

  AbsEnc_position2 = Absolute_Encoder_pos2 * 100;
  AbsEnc_position2_int32 = int(AbsEnc_position2);
  // int AbsEnc_position2_int32_HIGH = AbsEnc_position2_int32 >> 16;
  // int AbsEnc_position2_int32_LOW = AbsEnc_position2_int32-(AbsEnc_position2_int32_HIGH<<16);
  // int AbsEnc_position2_int32_HIGH_HIGH = AbsEnc_position2_int32_HIGH >> 8;
  // int AbsEnc_position2_int32_HIGH_LOW = AbsEnc_position2_int32_HIGH-(AbsEnc_position2_int32_HIGH_HIGH<<8);
  // int AbsEnc_position2_int32_LOW_HIGH = AbsEnc_position2_int32_LOW >> 8;
  // int AbsEnc_position2_int32_LOW_LOW = AbsEnc_position2_int32_LOW-(AbsEnc_position2_int32_LOW_HIGH<<8);

  // SensorCan1.Data[0] = byte(AbsEnc_position1_int32_HIGH_HIGH);
  // SensorCan1.Data[1] = byte(AbsEnc_position1_int32_HIGH_LOW);
  // SensorCan1.Data[2] = byte(AbsEnc_position1_int32_LOW_HIGH);
  // SensorCan1.Data[3] = byte(AbsEnc_position1_int32_LOW_LOW);
  // SensorCan1.Data[4] = byte(AbsEnc_position2_int32_HIGH_HIGH);
  // SensorCan1.Data[5] = byte(AbsEnc_position2_int32_HIGH_LOW);
  // SensorCan1.Data[6] = byte(AbsEnc_position2_int32_LOW_HIGH);
  // SensorCan1.Data[7] = byte(AbsEnc_position2_int32_LOW_LOW);

  SensorCan1.Data[0] = byteRead(AbsEnc_position1_int32, 3);
  SensorCan1.Data[1] = byteRead(AbsEnc_position1_int32, 2);
  SensorCan1.Data[2] = byteRead(AbsEnc_position1_int32, 1);
  SensorCan1.Data[3] = byteRead(AbsEnc_position1_int32, 0);
  SensorCan1.Data[4] = byteRead(AbsEnc_position2_int32, 3);
  SensorCan1.Data[5] = byteRead(AbsEnc_position2_int32, 2);
  SensorCan1.Data[6] = byteRead(AbsEnc_position2_int32, 1);
  SensorCan1.Data[7] = byteRead(AbsEnc_position2_int32, 0);

  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");


  CAN_EXTsend(SensorCan1);
  // Serial.println(":");

}
void RotEnc_CAN() {
  // Serial.print("RotEnc : ");
  // CanData SensorCanData; //Rotary_Encoder_pos
  // SensorCanData.Id = 0x2382;
  // SensorCanData.length = 8;
  
  // Rotary_Encoder_pos2 = 12345.67;   // HEX 00 12 D6 87
  // Rotary_Encoder_pos1 = -12345.67;  // HEX FF ED 29 79  
  
  RotEnc_position1 = Rotary_Encoder_pos1 * 100;
  RotEnc_position1_int32 = int(RotEnc_position1);
  // int RotEnc_position1_int32_HIGH = RotEnc_position1_int32 >> 16;
  // int RotEnc_position1_int32_LOW = RotEnc_position1_int32-(RotEnc_position1_int32_HIGH<<16);
  // int RotEnc_position1_int32_HIGH_HIGH = RotEnc_position1_int32_HIGH >> 8;
  // int RotEnc_position1_int32_HIGH_LOW = RotEnc_position1_int32_HIGH-(RotEnc_position1_int32_HIGH_HIGH<<8);
  // int RotEnc_position1_int32_LOW_HIGH = RotEnc_position1_int32_LOW >> 8;
  // int RotEnc_position1_int32_LOW_LOW = RotEnc_position1_int32_LOW-(RotEnc_position1_int32_LOW_HIGH<<8);

  RotEnc_position2 = Rotary_Encoder_pos2 * 100;
  RotEnc_position2_int32 = int(RotEnc_position2);
  // int RotEnc_position2_int32_HIGH = RotEnc_position2_int32 >> 16;
  // int RotEnc_position2_int32_LOW = RotEnc_position2_int32-(RotEnc_position2_int32_HIGH<<16);
  // int RotEnc_position2_int32_HIGH_HIGH = RotEnc_position2_int32_HIGH >> 8;
  // int RotEnc_position2_int32_HIGH_LOW = RotEnc_position2_int32_HIGH-(RotEnc_position2_int32_HIGH_HIGH<<8);
  // int RotEnc_position2_int32_LOW_HIGH = RotEnc_position2_int32_LOW >> 8;
  // int RotEnc_position2_int32_LOW_LOW = RotEnc_position2_int32_LOW-(RotEnc_position2_int32_LOW_HIGH<<8);

  // SensorCan2.Data[0] = byte(RotEnc_position1_int32_HIGH_HIGH);
  // SensorCan2.Data[1] = byte(RotEnc_position1_int32_HIGH_LOW);
  // SensorCan2.Data[2] = byte(RotEnc_position1_int32_LOW_HIGH);
  // SensorCan2.Data[3] = byte(RotEnc_position1_int32_LOW_LOW);
  // SensorCan2.Data[4] = byte(RotEnc_position2_int32_HIGH_HIGH);
  // SensorCan2.Data[5] = byte(RotEnc_position2_int32_HIGH_LOW);
  // SensorCan2.Data[6] = byte(RotEnc_position2_int32_LOW_HIGH);
  // SensorCan2.Data[7] = byte(RotEnc_position2_int32_LOW_LOW);

  SensorCan2.Data[0] = byteRead(RotEnc_position1_int32, 3);
  SensorCan2.Data[1] = byteRead(RotEnc_position1_int32, 2);
  SensorCan2.Data[2] = byteRead(RotEnc_position1_int32, 1);
  SensorCan2.Data[3] = byteRead(RotEnc_position1_int32, 0);
  SensorCan2.Data[4] = byteRead(RotEnc_position2_int32, 3);
  SensorCan2.Data[5] = byteRead(RotEnc_position2_int32, 2);
  SensorCan2.Data[6] = byteRead(RotEnc_position2_int32, 1);
  SensorCan2.Data[7] = byteRead(RotEnc_position2_int32, 0);

  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");

  CAN_EXTsend(SensorCan2);
  // Serial.println(":");

}
void RS422Enc_CAN() {
  // Serial.print("422Enc : ");
  // CanData SensorCanData; //RS422_Encoder_pos
  // SensorCanData.Id = 0x2383;
  // SensorCanData.length = 8;
  
  // RS422_Encoder_pos1 = 12345.67;   // HEX 00 12 D6 87
  // RS422_Encoder_pos2 = -12345.67;  // HEX FF ED 29 79  

  // float Enc_position1 = RS422_Encoder_pos1 * 100;
  // int Enc_position1_int32 = int(Enc_position1);
  // int Enc_position1_int32_HIGH = Enc_position1_int32 >> 16;
  // int Enc_position1_int32_LOW = Enc_position1_int32-(Enc_position1_int32_HIGH<<16);
  // int Enc_position1_int32_HIGH_HIGH = Enc_position1_int32_HIGH >> 8;
  // int Enc_position1_int32_HIGH_LOW = Enc_position1_int32_HIGH-(Enc_position1_int32_HIGH_HIGH<<8);
  // int Enc_position1_int32_LOW_HIGH = Enc_position1_int32_LOW >> 8;
  // int Enc_position1_int32_LOW_LOW = Enc_position1_int32_LOW-(Enc_position1_int32_LOW_HIGH<<8);

  // float Enc_position2 = RS422_Encoder_pos2 * 100;
  // int Enc_position2_int32 = int(Enc_position2);
  // int Enc_position2_int32_HIGH = Enc_position2_int32 >> 16;
  // int Enc_position2_int32_LOW = Enc_position2_int32-(Enc_position2_int32_HIGH<<16);
  // int Enc_position2_int32_HIGH_HIGH = Enc_position2_int32_HIGH >> 8;
  // int Enc_position2_int32_HIGH_LOW = Enc_position2_int32_HIGH-(Enc_position2_int32_HIGH_HIGH<<8);
  // int Enc_position2_int32_LOW_HIGH = Enc_position2_int32_LOW >> 8;
  // int Enc_position2_int32_LOW_LOW = Enc_position2_int32_LOW-(Enc_position2_int32_LOW_HIGH<<8);

  SensorCan3.Data[0] = data.buffer1;
  SensorCan3.Data[1] = data.buffer2;
  SensorCan3.Data[2] = data.buffer3;
  SensorCan3.Data[3] = 0x00;
  SensorCan3.Data[4] = 0x00;
  // SensorCan3.Data[5] = data2.buffer1;
  // SensorCan3.Data[6] = data2.buffer2;
  // SensorCan3.Data[7] = data2.buffer3;
  SensorCan3.Data[5] = 0x00;
  SensorCan3.Data[6] = 0x00;
  SensorCan3.Data[7] = 0x00;

  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");

  CAN_EXTsend(SensorCan3);
  // Serial.println(":");

}
void LoadCell_CAN() {
  // Serial.print("LoadCell : ");
  // CanData SensorCanData; //LoadCell_value
  // SensorCanData.Id = 0x2384;
  // SensorCanData.length = 8;
  
  // LoadCell_value1 = 12345.67;   // HEX 00 12 D6 87
  // LoadCell_value2 = -12345.67;  // HEX FF ED 29 79  

  loadcell_degree1 = LoadCell_value1 * 100;
  loadcell_degree1_int32 = int(loadcell_degree1);
  // int loadcell_degree1_int32_HIGH = loadcell_degree1_int32 >> 16;
  // int loadcell_degree1_int32_LOW = loadcell_degree1_int32-(loadcell_degree1_int32_HIGH<<16);
  // int loadcell_degree1_int32_HIGH_HIGH = loadcell_degree1_int32_HIGH >> 8;
  // int loadcell_degree1_int32_HIGH_LOW = loadcell_degree1_int32_HIGH-(loadcell_degree1_int32_HIGH_HIGH<<8);
  // int loadcell_degree1_int32_LOW_HIGH = loadcell_degree1_int32_LOW >> 8;
  // int loadcell_degree1_int32_LOW_LOW = loadcell_degree1_int32_LOW-(loadcell_degree1_int32_LOW_HIGH<<8);

  loadcell_degree2 = LoadCell_value2 * 100;
  loadcell_degree2_int32 = int(loadcell_degree2);
  // int loadcell_degree2_int32_HIGH = loadcell_degree2_int32 >> 16;
  // int loadcell_degree2_int32_LOW = loadcell_degree2_int32-(loadcell_degree2_int32_HIGH<<16);
  // int loadcell_degree2_int32_HIGH_HIGH = loadcell_degree2_int32_HIGH >> 8;
  // int loadcell_degree2_int32_HIGH_LOW = loadcell_degree2_int32_HIGH-(loadcell_degree2_int32_HIGH_HIGH<<8);
  // int loadcell_degree2_int32_LOW_HIGH = loadcell_degree2_int32_LOW >> 8;
  // int loadcell_degree2_int32_LOW_LOW = loadcell_degree2_int32_LOW-(loadcell_degree2_int32_LOW_HIGH<<8);

  // SensorCan4.Data[0] = byte(loadcell_degree1_int32_HIGH_HIGH);
  // SensorCan4.Data[1] = byte(loadcell_degree1_int32_HIGH_LOW);
  // SensorCan4.Data[2] = byte(loadcell_degree1_int32_LOW_HIGH);
  // SensorCan4.Data[3] = byte(loadcell_degree1_int32_LOW_LOW);
  // SensorCan4.Data[4] = byte(loadcell_degree2_int32_HIGH_HIGH);
  // SensorCan4.Data[5] = byte(loadcell_degree2_int32_HIGH_LOW);
  // SensorCan4.Data[6] = byte(loadcell_degree2_int32_LOW_HIGH);
  // SensorCan4.Data[7] = byte(loadcell_degree2_int32_LOW_LOW);

  SensorCan4.Data[0] = byteRead(loadcell_degree1_int32, 3);
  SensorCan4.Data[1] = byteRead(loadcell_degree1_int32, 2);
  SensorCan4.Data[2] = byteRead(loadcell_degree1_int32, 1);
  SensorCan4.Data[3] = byteRead(loadcell_degree1_int32, 0);
  SensorCan4.Data[4] = byteRead(loadcell_degree2_int32, 3);
  SensorCan4.Data[5] = byteRead(loadcell_degree2_int32, 2);
  SensorCan4.Data[6] = byteRead(loadcell_degree2_int32, 1);
  SensorCan4.Data[7] = byteRead(loadcell_degree2_int32, 0);
  // Serial.println(" ");
  // for (int i = 0; i <= 7; i++) {
  //   Serial.print(SensorCanData.Data[i], HEX);
  //   Serial.print(" ");
  // }
  // Serial.println(" ");

  CAN_EXTsend(SensorCan4);
  // Serial.println(":");

}
