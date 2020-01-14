#include <ASTCanLib.h>
#include <Timer.h>

#define MESSAGE_ID        0       // Message ID
#define MESSAGE_PROTOCOL  1       // CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
#define MESSAGE_LENGTH    8       // Data length: 8 bytes
#define MESSAGE_RTR       0       // rtr bit

#define NUMBER_OF_DIGIT   3

// Function prototypes
void processCan();
void serialPrintData(st_cmd_t *msg);
void displayNumber(int toDisplay);
void displayValue();
void lightNumber(int numberToDisplay);

int getNMEA2Kdata(st_cmd_t *msg);

// CAN message object
st_cmd_t Msg;
// Buffer for CAN data
uint8_t Buffer[8] = {};

Timer t;
int Depth = 0;  // /10 m

// pin mapping 7seg
int digit1 = 5; //PWM Display pin 2, second digit
int digit2 = 6; //PWM Display pin 6, third digit
int digit3 = 7; //PWM Display pin 8, fourth digit
int segA = 10; //Display pin 14
int segB = 11; //Display pin 16
int segC = 12; //Display pin 13
int segD = 13; //Display pin 3
int segE = 14; //Display pin 5
int segF = 15; //Display pin 11
int segG = 16; //Display pin 15

void setup() 
{    
  // 7seg LEDs
  pinMode(segA, OUTPUT);  pinMode(segB, OUTPUT);  pinMode(segC, OUTPUT);  pinMode(segD, OUTPUT);  
  pinMode(segE, OUTPUT);  pinMode(segF, OUTPUT);  pinMode(segG, OUTPUT);  
  pinMode(digit1, OUTPUT);  pinMode(digit2, OUTPUT);  pinMode(digit3, OUTPUT);

  // CAN
  canInit(250000);            // Initialise CAN port 250kbps. must be before Serial.begin
  Serial.begin(1000000);       // start serial port 1Mbps
  Msg.pt_data = &Buffer[0];    // reference message data to buffer
  
  // Initialise CAN packet.
  // All of these will be overwritten by a received packet
  Msg.ctrl.ide = MESSAGE_PROTOCOL;  // Set CAN protocol (0: CAN 2.0A, 1: CAN 2.0B)
  Msg.id.ext   = MESSAGE_ID;        // Set message ID
  Msg.dlc      = MESSAGE_LENGTH;    // Data length: 8 bytes
  Msg.ctrl.rtr = MESSAGE_RTR;       // Set rtr bit

  int tickEvent = t.every(15, displayValue, 0);

  displayNumber(Depth);
}

void loop() 
{
    
  t.update();

  processCan();
  
}

void displayValue()
{
  //displayNumber(millis() / 1000);
  displayNumber(Depth);
}

void processCan()
{
  
  clearBuffer(&Buffer[0]);  // Clear the message buffer
  Msg.cmd = CMD_RX_DATA;  // Send command to the CAN port controller

  // Wait for the command to be accepted by the controller
  while (can_cmd(&Msg) != CAN_CMD_ACCEPTED)
  {
    t.update();
  }
  // Wait for command to finish executing
  while (can_get_status(&Msg) == CAN_STATUS_NOT_COMPLETED)
  {
    t.update();
  }

  Depth = Msg.pt_data[3];

  getNMEA2Kdata(&Msg);
  serialPrintData(&Msg);

  
}

int getNMEA2Kdata(st_cmd_t *msg)
{
  char textBuffer[50] = {0};
  Serial.print("NMEA2k\r\n");
  
  sprintf(textBuffer, "id ext %08x", msg->id.ext);
  
  if (msg->id.ext == 0x00000b23)  // depth 
  {
    //msg->pt_data[i]
    Serial.print("depth\r\n");
  }
  else if (msg->id.ext == 0x00000323) // speed
  {
    Serial.print("speed\r\n");    
  }
  else if (msg->id.ext == 0x00000723) // water temp
  {
    Serial.print("water temp\r\n");
  }

  
  else if (textBuffer[3] == '1' && textBuffer[2] == '2') // heading 0x1223 = 4643
  {
    Serial.print("heading\r\n");
  }
  
}

void serialPrintData(st_cmd_t *msg) {
  char textBuffer[50] = {0};
  if (msg->ctrl.ide > 0) 
  {
    sprintf(textBuffer, "id ext %08x ", msg->id.ext);
  }
  else
  {
    sprintf(textBuffer, "id std %04x ", msg->id.std);
  }
  Serial.print(textBuffer);

  //  IDE
  sprintf(textBuffer, "ide %d ", msg->ctrl.ide);
  Serial.print(textBuffer);
  //  RTR
  sprintf(textBuffer, "rtr %d ", msg->ctrl.rtr);
  Serial.print(textBuffer);
  //  DLC
  sprintf(textBuffer, "dlc %d ", msg->dlc);
  Serial.print(textBuffer);
  //  Data
  sprintf(textBuffer, "data ");
  Serial.print(textBuffer);

  for (int i = 0; i < msg->dlc; i++) {
    sprintf(textBuffer, "%02X ", msg->pt_data[i]);
    Serial.print(textBuffer);
  }
  Serial.print("\r\n");
}


void displayNumber(int toDisplay) {
#define DISPLAY_BRIGHTNESS  500
#define DIGIT_ON  HIGH
#define DIGIT_OFF  LOW
  long beginTime = millis();

  for (int digit = NUMBER_OF_DIGIT ; digit > 0 ; digit--) 
  {
    //Turn on a digit for a short amount of time
    switch (digit) 
    {
      case 1:
        digitalWrite(digit1, DIGIT_ON);
        break;
      case 2:
        digitalWrite(digit2, DIGIT_ON);
        break;
      case 3:
        digitalWrite(digit3, DIGIT_ON);
        break;
    }
    //Turn on the right segments for this digit
    lightNumber(toDisplay % 10);
    toDisplay /= 10;
    delayMicroseconds(DISPLAY_BRIGHTNESS); //Display this digit for a fraction of a second (between 1us and 5000us, 500 is pretty good)
    //Turn off all segments
    lightNumber(10);
    //Turn off all digits
    digitalWrite(digit1, DIGIT_OFF);
    digitalWrite(digit2, DIGIT_OFF);
    digitalWrite(digit3, DIGIT_OFF);
  }
  while ( (millis() - beginTime) < 10) ; //Wait for 20ms to pass before we paint the display again
}

//Given a number, turns on those segments
//If number == 10, then turn off number
void lightNumber(int numberToDisplay) {
#define SEGMENT_ON  LOW
#define SEGMENT_OFF HIGH

  switch (numberToDisplay) 
  {
    case 0:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_ON);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_OFF);
      break;
    case 1:
      digitalWrite(segA, SEGMENT_OFF);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_OFF);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_OFF);
      digitalWrite(segG, SEGMENT_OFF);
      break;
    case 2:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_OFF);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_ON);
      digitalWrite(segF, SEGMENT_OFF);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 3:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_OFF);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 4:
      digitalWrite(segA, SEGMENT_OFF);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_OFF);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 5:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_OFF);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 6:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_OFF);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_ON);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 7:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_OFF);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_OFF);
      digitalWrite(segG, SEGMENT_OFF);
      break;
    case 8:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_ON);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 9:
      digitalWrite(segA, SEGMENT_ON);
      digitalWrite(segB, SEGMENT_ON);
      digitalWrite(segC, SEGMENT_ON);
      digitalWrite(segD, SEGMENT_ON);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_ON);
      digitalWrite(segG, SEGMENT_ON);
      break;
    case 10:
      digitalWrite(segA, SEGMENT_OFF);
      digitalWrite(segB, SEGMENT_OFF);
      digitalWrite(segC, SEGMENT_OFF);
      digitalWrite(segD, SEGMENT_OFF);
      digitalWrite(segE, SEGMENT_OFF);
      digitalWrite(segF, SEGMENT_OFF);
      digitalWrite(segG, SEGMENT_OFF);
      break;
  }
}
