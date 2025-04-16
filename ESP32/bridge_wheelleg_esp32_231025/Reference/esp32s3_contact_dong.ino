// [Serial2]
const int rx2 = 2;
const int tx2 = 42;

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
const int PIN_TURRET_PWR1 = 10;
const int PIN_TURRET_PWR2 = 11;
const int PIN_TURRET_DIR1 = 12;
const int PIN_TURRET_DIR2 = 9;
const int PIN_TURRET_ON1 = 13;
const int PIN_TURRET_ON2 = 14;

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
  // xTaskCreatePinnedToCore(
  //   TASK1,         // task function
  //   "Task1",           // task name
  //   10000,             // stack size (word)
  //   NULL,              // task parameters
  //   0,                 // task priority
  //   &Task1,            // task handle
  //   0); 
  
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial2.setTimeout(10);

  
  pinMode(pin_ct[0],INPUT_PULLUP);
  pinMode(pin_ct[1],INPUT_PULLUP);
  
  pinMode(RotaryEncoder1PinA, INPUT_PULLUP);
  pinMode(RotaryEncoder1PinB, INPUT_PULLUP);
  pinMode(RotaryEncoder2PinA, INPUT_PULLUP);
  pinMode(RotaryEncoder2PinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RotaryEncoder1PinA), updateRotaryEncoder1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RotaryEncoder2PinA), updateRotaryEncoder2, CHANGE);


}

void loop() {
  // put your main code here, to run repeatedly:
  for (int i=0;i<2;i++) {
    contact_bool[i] = digitalRead(pin_ct[i]);
    // Serial.println("contact"+String(i)+": "+String(contact[i]));
    // Serial.println(contact_bool[i]);
    if (contact_bool[i] == 0) {
      // count_0[i]++;
      // if (count_0[i] > 5) {
      //   contact[i] = 0;
      //   count_0[i] = 0;
      // }
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
  // Serial.println("encoder1: "+String(RotaryEncoderPos1)+" , "+"encoder2: "+String(RotaryEncoderPos2));
  vTaskDelay(pdMS_TO_TICKS(1));
  tx_to_esp();
  vTaskDelay(pdMS_TO_TICKS(1));
}

//=============Functions==============
void tx_to_esp()
{ 
  float checksum = contact[0] + contact[1] + RotaryEncoderPos1*0.06 + RotaryEncoderPos2*0.06;
  String msg_to_esp = "SRT";
  msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(contact[0]); 
  msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(contact[1]);
  msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(RotaryEncoderPos1*0.06);
  msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(RotaryEncoderPos2*0.06);
  msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + String(checksum);
  msg_to_esp = msg_to_esp + ",";
  msg_to_esp = msg_to_esp + "END";
  Serial2.println(msg_to_esp);
  Serial.println(msg_to_esp);
  // Serial.print(" , ");
}

//=============MULTITASK FUNCTIONS===============
// void TASK1(void * parameter) {
//   for(;;) {
//     contact_sensor();
//   }
// }

// void contact_sensor() {
//   contact[0] = digitalRead(pin_ct);
//   Serial.println(LoadCell_value1);
//   vTaskDelay(pdMS_TO_TICKS(1));
// }
