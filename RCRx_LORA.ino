#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>
#include "HT_SSD1306Wire.h"
#include <ESP32Servo.h>

#define MAX_STEER 160
#define MIN_STEER 20

#define STEER_PIN 26
#define ESC_PIN 19
#define PAN_PIN 4
#define TILT_PIN 33

#define RF_FREQUENCY 915000000 // Hz

#define TX_OUTPUT_POWER 100 // dBm

#define LORA_BANDWIDTH 1        // [0: 125 kHz,
                                //  1: 250 kHz,
                                //  2: 500 kHz,
                                //  3: Reserved]
#define LORA_SPREADING_FACTOR 7 // [SF7..SF12]
#define LORA_CODINGRATE 1       // [1: 4/5,
                                //  2: 4/6,
                                //  3: 4/7,
                                //  4: 4/8]
#define LORA_PREAMBLE_LENGTH 4  // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 4   // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false

#define RX_TIMEOUT 500
#define BUFFER_SIZE 4                                                                 // Define the payload size here
SSD1306Wire oledDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

static RadioEvents_t RadioEvents;

// data structure of device
struct State
{
  int steer;
  int esc;
  int pan;
  int tilt;
  bool auxButton[8];
};

Servo steer;
Servo esc;
Servo pan;
Servo tilt;

int16_t rssi, rxSize;
uint8_t buffer[BUFFER_SIZE];
char str1[30];
char str2[20];
char str3[30];
char str4[40];

unsigned long lastEventTime;
bool timeOut = false;

bool lora_idle = true;

void VextON(void)
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) // Vext default OFF
{
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, HIGH);
}

void calibrateESC()
{
  oledDisplay.drawString(0, 10, "Calibrating ESC...");
  oledDisplay.display();
  Serial.println("FULL");
  esc.write(180);
  delay(5000);
  Serial.println("NONE");
  esc.write(0);
  delay(5000);
  Serial.println("CENTER");
  esc.write(90);
  delay(5000);
  oledDisplay.drawString(0, 20, "Calibration Complete");
  oledDisplay.display();
}

void setup()
{
  Serial.begin(115200);
  VextON();
  oledDisplay.init();
  oledDisplay.setFont(ArialMT_Plain_10);
  oledDisplay.clear();
  oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
  oledDisplay.drawString(0, 0, "Starting Up...");
  oledDisplay.display();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  steer.setPeriodHertz(50); // Standard 50hz servo
  esc.setPeriodHertz(50);   // Standard 50hz servo
  pan.setPeriodHertz(50);
  tilt.setPeriodHertz(50);

  
  steer.attach(STEER_PIN, 1000, 2000); // Min: 600 Max: 2000 for pan/tilt servo
  esc.attach(ESC_PIN, 600, 2400);
  pan.attach(PAN_PIN, 600,2000);
  tilt.attach(TILT_PIN, 600,2000);

  steer.write(90);
  pan.write(90);
  tilt.write(90);
  calibrateESC();
  esc.write(90);

  Mcu.begin();

  rssi = 0;

  RadioEvents.RxDone = OnRxDone;
  RadioEvents.RxTimeout = OnRxTimeout;
  RadioEvents.RxError = OnRxError;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
  oledDisplay.clear();
  oledDisplay.drawString(0, 10, "Start First RX...");
  oledDisplay.display();
  unsigned long lastEventTime = millis();
}

void describeState(State state){
    Serial.println("******************************");
    Serial.print("Steer: ");
    Serial.println(state.steer);
    Serial.print("Esc: ");
    Serial.println(state.esc);
    Serial.print("Pan: ");
    Serial.println(state.pan);
    Serial.print("Tilt: ");
    Serial.println(state.tilt);

    for(int i=0; i<8; i++){
        Serial.print("Button ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(state.auxButton[i]);
    }
    Serial.println("******************************");
}

State readState(uint8_t buffer[BUFFER_SIZE]){

    State newState;
    newState.steer = buffer[0];
    newState.esc = buffer[1];

    newState.tilt = map(buffer[2] & 15, 0, 15, 0, 180);//Get bottom 4 bits
    newState.pan = map(buffer[2] >> 4, 0, 15, 0, 180);//Shift top 4 bits right and take bottom value

    for(int i=0; i<8;i++){
        int mask = pow(2,i);
        if(buffer[3] & mask){
            newState.auxButton[i] = true;
        }
    }
    //describeState(newState);
    return newState;
}

void applyState(State state){
    int steerValue;
    if(state.steer > MAX_STEER){
        steerValue = MAX_STEER;
    }else if(state.steer < MIN_STEER){
        steerValue = MIN_STEER;
    }else{
        steerValue = state.steer;
    }

    
    sprintf(str3, "Steer: %d ESC: %d", steerValue, state.esc);
    sprintf(str4, "Pan: %d Tilt: %d", state.pan, state.tilt);
    oledDisplay.drawString(0, 20, str3);
    oledDisplay.drawString(0, 30, str4);

    lastEventTime = millis();
    steer.write(steerValue);
    esc.write(state.esc);
    pan.write(state.pan);
    tilt.write(state.tilt);
}

void shutdownServos(){
    Serial.println("Shutting down servos");
    buffer[0] = 90;
    buffer[1] = 90;
    buffer[2] = 0;
    buffer[3] = 0;
    applyState(readState(buffer));
}

void loop()
{
  unsigned long curTime = millis();
  if(curTime > lastEventTime + RX_TIMEOUT && !timeOut){
    timeOut = true;
    oledDisplay.clear();
    oledDisplay.drawString(0, 0, "RX: TIMED OUT");
    shutdownServos();
    oledDisplay.display();
  }
  
  if (lora_idle)
  {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr)
{
  rssi = rssi;
  rxSize = size;
  memcpy(buffer, payload, size);
  Radio.Sleep();
  Serial.printf("\r\nreceived packet - (rssi: %d) \"%d\", \"%d\", \"%d\", \"%d\"\r\n", rssi, buffer[0], buffer[1], buffer[2], buffer[3]);

  sprintf(str1, "RX: %d %d %d %d", buffer[0], buffer[1], buffer[2], buffer[3]);
  sprintf(str2, "RSSI: %d", rssi);
  oledDisplay.clear();
  oledDisplay.drawString(0, 0, str1);
  oledDisplay.drawString(0, 10, str2);
  applyState(readState(buffer));
  oledDisplay.display();
  timeOut = false;
  lora_idle = true;
}

void OnRxTimeout(){
  Radio.Sleep();
  Serial.println("RX: TIMED OUT");
  oledDisplay.clear();
  oledDisplay.drawString(0, 0, "RX: TIMED OUT");
  shutdownServos();
  oledDisplay.display();
  lora_idle=true;
}

void OnRxError(){
  Radio.Sleep();
  Serial.println("RX: ERROR");
  oledDisplay.clear();
  oledDisplay.drawString(0, 0, "RX: ERROR");
  shutdownServos();
  oledDisplay.display();
  lora_idle=true;
}
