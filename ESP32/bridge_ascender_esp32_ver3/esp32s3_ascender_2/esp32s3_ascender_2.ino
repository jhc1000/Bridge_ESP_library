#include <HX711.h>

// [Serial1]
//define what pin rx2 tx2 is going to be
#define rx1 2                                        
#define tx1 42

// [Serial2]
//define what pin rx2 tx2 is going to be
#define rx2 47                                          
#define tx2 21
unsigned long timeout = 50; //50ms

// [LoadCell]
#define calibration_factor1 -2180
#define dtout1 17  //DT 전선 연결
#define clk1 16    // SCLK 전선 연결
HX711 scale1;
float LoadCell_value1 = 0.0;

// [Encoder]
// [Rotary]
const int RotaryEncoder1PinA = 14;
const int RotaryEncoder1PinB = 13;
// const int RotaryEncoder2PinA = 17;
// const int RotaryEncoder2PinB = 16;
volatile int RotaryEncoderPos1 = 0;
int RotaryEncoderPinALast1 = LOW;
// volatile int RotaryEncoderPos2 = 0;
// int RotaryEncoderPinALast2 = LOW;
float Rotary_Encoder_pos1 = 0.0;

// [Absolute]
int AbsoluteEncoderPin1[10]{ 4, 5, 6, 7, 15, 18, 8, 9, 10, 11};
int AbsoluteEncoder1[10];
float AbsoluteEncoderPos1 = 0.0;
// int AbsoluteEncoderPin2[10]{ 36, 35, 1, 2, 42, 41, 40, 39, 38, 37};
// int AbsoluteEncoder2[10];
// int AbsoluteEncoderPos2 = 0;
float Absolute_Encoder_pos1 = 0.0;

// [RS422 abs Encoder]
struct SensorData {
  byte buffer1;
  byte buffer2;
  byte buffer3;
}data;
float RS422_Encoder_pos = 0.0;

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

// void IRAM_ATTR updateRotaryEncoder2() {  //IRAM_ATTR call interupt function
//   int RotaryEncoderPinAState2 = digitalRead(RotaryEncoder2PinA);

//   if (RotaryEncoderPinAState2 != RotaryEncoderPinALast2) {
//     if (digitalRead(RotaryEncoder2PinB) != RotaryEncoderPinAState2) {
//       RotaryEncoderPos2++;
//     } else {
//       RotaryEncoderPos2--;
//     }
//   }

//   RotaryEncoderPinALast2 = RotaryEncoderPinAState2;
// }

// [Functions]
void tx_to_esp();
float rx_from_RS422();
void rx_from_loadcell();

// [Multitask]
TaskHandle_t Task1;

void setup() {
  xTaskCreatePinnedToCore(
    TASK1,         // task function
    "Task1",           // task name
    10000,             // stack size (word)
    NULL,              // task parameters
    0,                 // task priority
    &Task1,            // task handle
    0);                // exe core

  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, rx1, tx1);
  Serial2.begin(115200, SERIAL_8N1, rx2, tx2);
  Serial1.setTimeout(timeout);
  Serial2.setTimeout(timeout);

  pinMode(RotaryEncoder1PinA, INPUT_PULLUP);
  pinMode(RotaryEncoder1PinB, INPUT_PULLUP);
  // pinMode(RotaryEncoder2PinA, INPUT_PULLUP);
  // pinMode(RotaryEncoder2PinB, INPUT_PULLUP);

  pinMode(AbsoluteEncoderPin1[0], INPUT_PULLUP);
  pinMode(AbsoluteEncoderPin1[1], INPUT_PULLUP);
  pinMode(AbsoluteEncoderPin1[2], INPUT_PULLUP);
  pinMode(AbsoluteEncoderPin1[3], INPUT_PULLUP);
  pinMode(AbsoluteEncoderPin1[4], INPUT_PULLUP);
  pinMode(AbsoluteEncoderPin1[5], INPUT_PULLUP);
  pinMode(AbsoluteEncoderPin1[6], INPUT_PULLUP);
  pinMode(AbsoluteEncoderPin1[7], INPUT_PULLUP);
  pinMode(AbsoluteEncoderPin1[8], INPUT_PULLUP);
  pinMode(AbsoluteEncoderPin1[9], INPUT_PULLUP);

  // pinMode(AbsoluteEncoderPin2[0], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin2[1], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin2[2], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin2[3], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin2[4], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin2[5], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin2[6], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin2[7], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin2[8], INPUT_PULLUP);
  // pinMode(AbsoluteEncoderPin2[9], INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RotaryEncoder1PinA), updateRotaryEncoder1, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(RotaryEncoder2PinA), updateRotaryEncoder2, CHANGE);

  scale1.begin(dtout1, clk1);
  scale1.set_scale(calibration_factor1);
  scale1.tare();

  // scale2.begin(dtout2, clk2);
  // scale2.set_scale(calibration_factor2);
  // scale2.tare();
}

void loop() {

  AbsoluteEncoderPos1 = 0;
  // AbsoluteEncoderPos2 = 0;

  for (int i = 1; i <= 10; i++) {
    AbsoluteEncoder1[i] = digitalRead(AbsoluteEncoderPin1[i]);
  }

  for (int i = 0; i <= 9; i++) {
    AbsoluteEncoderPos1 = AbsoluteEncoderPos1 + AbsoluteEncoder1[i] * pow(2, i);
    //Serial.print(AbsoluteEncoder1[i]);
  }

  // for (int i = 1; i <= 10; i++) {
  //  AbsoluteEncoder2[i] = digitalRead(AbsoluteEncoderPin2[i]);
  // }

  // for (int i = 0; i <= 9; i++) {
  //  AbsoluteEncoderPos2 = AbsoluteEncoderPos2 + AbsoluteEncoder2[i] * pow(2, i);
  //  //Serial.print(AbsoluteEncoder2[i]);
  // }

  // LoadCell_value1 = scale1.get_units();
  // readings2 = scale2.get_units();

  rx_from_RS422();

  vTaskDelay(pdMS_TO_TICKS(1));

  tx_to_esp();
  // vTaskDelay(pdMS_TO_TICKS(1));

  //Serial.print(" , ");
  // Serial.print(RotaryEncoderPos1 * 0.06);
  // Serial.print(" , ");
  // Serial.print(RotaryEncoderPos2 * 0.06);
  // Serial.print(" , ");
  // Serial.print(AbsoluteEncoderPos1 * 360 / 1024);
  // Serial.print(" , ");
  // Serial.print(AbsoluteEncoderPos2 * 360 / 1024);
  // Serial.println(" , ");
  // Serial.println(10 * readings1, 3);
  // Serial.println(10 * readings2, 3);
}

//======================================================
void tx_to_esp()
{
  String msg_to_esp = "SRT";
  msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(RotaryEncoderPos1 * 0.06); 
  msg_to_esp = msg_to_esp + ",";
  // msg_to_esp = msg_to_esp + String(RotaryEncoderPos2 * 0.06); 
  // msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(AbsoluteEncoderPos1 * 360 / 1024);
  msg_to_esp = msg_to_esp + ",";
  // msg_to_esp = msg_to_esp + String(AbsoluteEncoderPos2 * 360 / 1024);
  // msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(data.buffer1);
  msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(data.buffer2);
  msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(data.buffer3);
  msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(LoadCell_value1);
  msg_to_esp = msg_to_esp + ",";
  // msg_to_esp = msg_to_esp + String(10 * readings2, 3);
  // msg_to_esp = msg_to_esp + ",";    
  msg_to_esp = msg_to_esp + "END";
  Serial2.print(msg_to_esp);
  Serial.println(msg_to_esp);
  // Serial.print(" , ");
}

float rx_from_RS422() {
  Serial1.write("3");
  vTaskDelay(pdMS_TO_TICKS(1));
  if(Serial1.available()>0){
    Serial1.readBytes((uint8_t*)&data, sizeof(SensorData));

    float AbsoluteEncoderPos = (float)((data.buffer3>>5)+(data.buffer1<<11)+(data.buffer2<<3))*360/524288;

    // Serial.print("Recieved RS422 value : ");
    // Serial.print(data.buffer1, HEX);
    // Serial.print("  ");
    // Serial.print(data.buffer2, HEX);
    // Serial.print("  ");
    // Serial.print(data.buffer3, HEX);
    // Serial.print("  ");
    // Serial.println(AbsoluteEncoderPos);
    return AbsoluteEncoderPos;
  }
  else {
    // Serial.println("==== RS422 not Received ===");
    return 0;
  }
}

//=============MULTITASK FUNCTIONS===============
void TASK1(void * parameter) {
  for(;;) {
    rx_from_loadcell();
  }
}

void rx_from_loadcell() {
  LoadCell_value1 = scale1.get_units();
  // Serial.println(LoadCell_value1);
  vTaskDelay(pdMS_TO_TICKS(1));
}
