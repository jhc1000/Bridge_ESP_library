#include <CAN.h>
// [CAN]
// define CAN struct
struct CanData {
  int Id;
  int length;
  int Extened;
  int RTR;
  byte Data[8];
};
CanData ReceiveCanData;
// pin select
int rx = 18;
int tx = 19;

void send_CAN_from_Serial();
void CAN_send(CanData InputCanData);
void tx_CAN_to_Motor(int ID);
void print_CAN_HEX();

// [Multitask]
TaskHandle_t Task1;
// Starting time of multitask

void setup() {

  xTaskCreatePinnedToCore(
    CANRECEIVE,         // task function
    "Task1",           // task name
    10000,             // stack size (word)
    NULL,              // task parameters
    0,                 // task priority
    &Task1,            // task handle
    0);                // exe core

  // put your setup code here, to run once:
  Serial.begin(500000);
  CAN.setPins(rx, tx);

  if (!CAN.begin(1000E3)) {
    //Serial.println("Starting CAN failed!");
    while (1);
  }

}

void loop() {
  // 받는 코드를 1개만 하면 주기때문에 1개만 나오거나 여러 데이터가 혼합된 값이 나온다.
  send_CAN_from_Serial(); //t6018000000001F8003E8\x0D
  //vTaskDelay(0.001);
  print_CAN_HEX();
  //tx_CAN_to_Motor(601);
  //tx_CAN_to_Motor(1538);
  //delay(10);

//   Serial.print(ReceiveCanData.Id, HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.length, HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[0], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[1], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[2], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[3], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[4], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[5], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[6], HEX);
//   Serial.print(" ");
//   Serial.println(ReceiveCanData.Data[7], HEX);
}

CanData CAN_receive(int MaskId){
  int packetSize = CAN.parsePacket();
  //while(CAN.packetId() != MaskId) {
  //  packetSize = CAN.parsePacket();
  //}
  if (packetSize){ // is there CAN data received?
    int Id_identifier = CAN.packetId();
    //if (CAN.packetId() == MaskId) {
    // Serial.print("Received ");
    ReceiveCanData.length = packetSize;
    ReceiveCanData.Id = CAN.packetId();
    
    if(CAN.packetExtended()){
      ReceiveCanData.Extened = 1;
      // Serial.print("extended ");
    }
    else{
      ReceiveCanData.Extened = 0;
    }

    if (CAN.packetRtr()) {
      ReceiveCanData.RTR = 1;
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
    
  //}
  }
  else {
    //Serial.println("==== No CAN Received ====");
  }
  return ReceiveCanData;
  
}

void CAN_send(CanData InputCanData){
  // Serial.print("begin packet");
  CAN.beginPacket(InputCanData.Id);
  // Serial.print(" write");
  CAN.write(InputCanData.Data, InputCanData.length);
  // Serial.print(" end packet");
  CAN.endPacket();
  // Serial.println(" done");
}

void send_CAN_from_Serial() {
  // if(Serial.available()>0) { // t6018(08/FE/../../../../../..)\x0D
    //String msg_from_pc = Serial.readStringUntil('\x0D'); // CR
    // String msg_from_pc = Serial.readStringUntil('\r'); 
    String msg_from_pc = "t6018000000001F8003E8\r";
    if (msg_from_pc.startsWith("t")) {
      // Serial.println(msg_from_pc);
      String ID = msg_from_pc.substring(1, 4);
      String DLC = msg_from_pc.substring(4, 5);
      String Data_String[8];
      Data_String[0] = msg_from_pc.substring(5, 7);
      Data_String[1] = msg_from_pc.substring(7, 9);
      Data_String[2] = msg_from_pc.substring(9, 11);
      Data_String[3] = msg_from_pc.substring(11, 13);
      Data_String[4] = msg_from_pc.substring(13, 15);
      Data_String[5] = msg_from_pc.substring(15, 17);
      Data_String[6] = msg_from_pc.substring(17, 19);
      Data_String[7] = msg_from_pc.substring(19, 21);
      //char* p;

      CanData SendCanData;
      // SendCanData.Id = int(strtol(ID.c_str(), & p, 16));
      // SendCanData.length = int(strtol(DLC.c_str(), & p, 16));
      // SendCanData.Data[0] = int(strtol(Data_String[0].c_str(), & p, 16));
      // SendCanData.Data[1] = int(strtol(Data_String[1].c_str(), & p, 16));
      // SendCanData.Data[2] = int(strtol(Data_String[2].c_str(), & p, 16));
      // SendCanData.Data[3] = int(strtol(Data_String[3].c_str(), & p, 16));
      // SendCanData.Data[4] = int(strtol(Data_String[4].c_str(), & p, 16));
      // SendCanData.Data[5] = int(strtol(Data_String[5].c_str(), & p, 16));
      // SendCanData.Data[6] = int(strtol(Data_String[6].c_str(), & p, 16));
      // SendCanData.Data[7] = int(strtol(Data_String[7].c_str(), & p, 16));
      // Serial.println("string to hex");

      SendCanData.Id = int(strtol(ID.c_str(), NULL, 16));
      SendCanData.length = int(strtol(DLC.c_str(), NULL, 16));
      SendCanData.Data[0] = int(strtol(Data_String[0].c_str(), NULL, 16));
      SendCanData.Data[1] = int(strtol(Data_String[1].c_str(), NULL, 16));
      SendCanData.Data[2] = int(strtol(Data_String[2].c_str(), NULL, 16));
      SendCanData.Data[3] = int(strtol(Data_String[3].c_str(), NULL, 16));
      SendCanData.Data[4] = int(strtol(Data_String[4].c_str(), NULL, 16));
      SendCanData.Data[5] = int(strtol(Data_String[5].c_str(), NULL, 16));
      SendCanData.Data[6] = int(strtol(Data_String[6].c_str(), NULL, 16));
      SendCanData.Data[7] = int(strtol(Data_String[7].c_str(), NULL, 16));

      CAN_send(SendCanData); //601 8 00 00 00 00 1F 80 03 E8
      

      // Serial.print(ID);
      // Serial.print(" : ");
      // Serial.print(DLC);
      // Serial.print(" : ");
      // Serial.print(Data_String[0]);
      // Serial.print(" ");
      // Serial.print(Data_String[1]);
      // Serial.print(" ");
      // Serial.print(Data_String[2]);
      // Serial.print(" ");
      // Serial.print(Data_String[3]);
      // Serial.print(" ");
      // Serial.print(Data_String[4]);
      // Serial.print(" ");
      // Serial.print(Data_String[5]);
      // Serial.print(" ");
      // Serial.print(Data_String[6]);
      // Serial.print(" ");
      // Serial.print(Data_String[7]);
      // Serial.println(" ");

      // Serial.print(SendCanData.Id, HEX);
      // Serial.print(" : ");
      // Serial.print(SendCanData.length, HEX);
      // Serial.print(" : ");
      // Serial.print(SendCanData.Data[0], HEX);
      // Serial.print(" ");
      // Serial.print(SendCanData.Data[1], HEX);
      // Serial.print(" ");
      // Serial.print(SendCanData.Data[2], HEX);
      // Serial.print(" ");
      // Serial.print(SendCanData.Data[3], HEX);
      // Serial.print(" ");
      // Serial.print(SendCanData.Data[4], HEX);
      // Serial.print(" ");
      // Serial.print(SendCanData.Data[5], HEX);
      // Serial.print(" ");
      // Serial.print(SendCanData.Data[6], HEX);
      // Serial.print(" ");
      // Serial.print(SendCanData.Data[7], HEX);
      // Serial.println(" ");

    }
  // }
}

void tx_CAN_to_Motor(int ID) {
  // Serial.print("CAN data send to Motor ");
  CanData SendCanData;
  SendCanData.Id = int(strtol(String(ID).c_str(), NULL, 16)); //1537
  SendCanData.length = 8;

  float motor_position_degree = 0.0;
  float motor_position = motor_position_degree * 10000;
  int motor_position_int32 = int(motor_position);
  int motor_position_int32_HIGH = motor_position_int32 >> 16;
  int motor_position_int32_LOW = motor_position_int32-(motor_position_int32_HIGH<<16);
  int motor_position_int32_HIGH_HIGH = motor_position_int32_HIGH>>8;
  int motor_position_int32_HIGH_LOW = motor_position_int32_HIGH-(motor_position_int32_HIGH_HIGH<<8);
  int motor_position_int32_LOW_HIGH = motor_position_int32_LOW>>8;
  int motor_position_int32_LOW_LOW = motor_position_int32_LOW-(motor_position_int32_LOW_HIGH<<8);

  float motor_position_speed = 3*64*21;
  int16_t motor_position_speed_int16 = int16_t(motor_position_speed);
  int16_t motor_position_speed_int16_HIGH = motor_position_speed_int16>>8;
  int16_t motor_position_speed_int16_LOW = motor_position_speed_int16-(motor_position_speed_int16_HIGH<<8);

  float motor_position_acc = 10;
  int16_t motor_position_acc_int16 = int16_t(motor_position_acc);
  int16_t motor_position_acc_int16_HIGH = motor_position_acc_int16>>8;
  int16_t motor_position_acc_int16_LOW = motor_position_acc_int16-(motor_position_acc_int16_HIGH<<8);

  SendCanData.Data[0] = byte(motor_position_int32_HIGH_HIGH);
  SendCanData.Data[1] = byte(motor_position_int32_HIGH_LOW);
  SendCanData.Data[2] = byte(motor_position_int32_LOW_HIGH);
  SendCanData.Data[3] = byte(motor_position_int32_LOW_LOW);
  SendCanData.Data[4] = byte(motor_position_speed_int16_HIGH);
  SendCanData.Data[5] = byte(motor_position_speed_int16_LOW);
  SendCanData.Data[6] = byte(motor_position_acc_int16_HIGH);
  SendCanData.Data[7] = byte(motor_position_acc_int16_LOW);

  // Serial.print(SendCanData.Data[0], HEX);
  // Serial.print(" ");
  // Serial.print(SendCanData.Data[1], HEX);
  // Serial.print(" ");
  // Serial.print(SendCanData.Data[2], HEX);
  // Serial.print(" ");
  // Serial.print(SendCanData.Data[3], HEX);
  // Serial.print(" ");
  // Serial.print(SendCanData.Data[4], HEX);
  // Serial.print(" ");
  // Serial.print(SendCanData.Data[5], HEX);
  // Serial.print(" ");
  // Serial.print(SendCanData.Data[6], HEX);
  // Serial.print(" ");
  // Serial.print(SendCanData.Data[7], HEX);
  // Serial.print(" ");

  //00 00 00 00 1F 80 03 E8
  //Serial.print("....");
  CAN_send(SendCanData);
  //Serial.println(" done"); 
}

// void onReceive(int packetSize) {
//   // received a packet
//   Serial.print("Received ");

//   if (CAN.packetExtended()) {
//     Serial.print("extended ");
//   }

//   if (CAN.packetRtr()) {
//     // Remote transmission request, packet contains no data
//     Serial.print("RTR ");
//   }

//   Serial.print("packet with id 0x");
//   Serial.print(CAN.packetId(), HEX);

//   if (CAN.packetRtr()) {
//     //Serial.print(" and requested length ");
//     //Serial.println(CAN.packetDlc());
//   } else {
//     //Serial.print(" and length ");
//     //Serial.println(packetSize);

//     // only print packet data for non-RTR packets
//     while (CAN.available()) {
//       Serial.print((char)CAN.read());
//     }
//     //Serial.println();
//   }

//   Serial.println();
// }

void CANRECEIVE(void * parameter) {
  for(;;) {
    //Serial.print("receiving can");
    ReceiveCanData = CAN_receive(0x2901);
    //Serial.println(".... done");
    //vTaskDelay(0.001);
  }
}

void print_CAN_HEX() {

//  String Id = String(ReceiveCanData.Id, HEX);
//  String length = String(ReceiveCanData.length, HEX);
//  String Data0 = String(ReceiveCanData.Data[0], HEX);
//  String Data1 = String(ReceiveCanData.Data[1], HEX);
//  String Data2 = String(ReceiveCanData.Data[2], HEX);
//  String Data3 = String(ReceiveCanData.Data[3], HEX);
//  String Data4 = String(ReceiveCanData.Data[4], HEX);
//  String Data5 = String(ReceiveCanData.Data[5], HEX);
//  String Data6 = String(ReceiveCanData.Data[6], HEX);
//  String Data7 = String(ReceiveCanData.Data[7], HEX);
  
//  Serial.println("t"+ Id+ " "+ length+ " "+ Data0+ " "+ Data1+ " "+ Data2+ " "+ Data3+ " "+ Data4+ 
//  " "+ Data5+ " "+ Data6+ " "+ Data7);
  Serial.println(String(ReceiveCanData.Id, HEX)+ " "+ String(ReceiveCanData.length, HEX)+ " "+ 
  String(ReceiveCanData.Data[0], HEX)+ " "+ String(ReceiveCanData.Data[1], HEX)+ " "+ 
  String(ReceiveCanData.Data[2], HEX)+ " "+ String(ReceiveCanData.Data[3], HEX)+ " "+ 
  String(ReceiveCanData.Data[4], HEX)+ " "+ String(ReceiveCanData.Data[5], HEX)+ " "+ 
  String(ReceiveCanData.Data[6], HEX)+ " "+ String(ReceiveCanData.Data[7], HEX));
//  //  Serial.println("t"+ Id+ ","+ length+ ","+ Data0+ ","+ Data1+ ","+ Data2+ ","+ Data3+ ","+ Data4+ ","+ Data5+
//  //  ","+ Data6+ ","+ Data7);
//  // Serial.print("t"+ Id+ length+ Data0+ Data1+ Data2+ Data3+ Data4+ Data5+
//  // Data6+ Data7+ "\r");
  
//   Serial.print(ReceiveCanData.Id, HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.length, HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[0], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[1], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[2], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[3], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[4], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[5], HEX);
//   Serial.print(" ");
//   Serial.print(ReceiveCanData.Data[6], HEX);
//   Serial.print(" ");
//   Serial.println(ReceiveCanData.Data[7], HEX);
}
