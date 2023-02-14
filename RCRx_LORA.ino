#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <Wire.h>               
#include "HT_SSD1306Wire.h"

#define RF_FREQUENCY                                915000000 // Hz

#define TX_OUTPUT_POWER                             100        // dBm

#define LORA_BANDWIDTH                              2         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        4         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            100
#define BUFFER_SIZE                                 4 // Define the payload size here
SSD1306Wire  oledDisplay(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED); // addr , freq , i2c group , resolution , rst

static RadioEvents_t RadioEvents;

int16_t rssi,rxSize;
uint8_t buffer[BUFFER_SIZE];
char str1[30];
char str2[20];

bool lora_idle = true;

void VextON(void)
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, LOW);
}

void VextOFF(void) //Vext default OFF
{
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext, HIGH);
}

void setup() {
  Serial.begin(115200);
  VextON();
  oledDisplay.init();
  oledDisplay.setFont(ArialMT_Plain_10);
  oledDisplay.clear();
  oledDisplay.setTextAlignment(TEXT_ALIGN_LEFT);
  oledDisplay.drawString(0, 0, "Starting Up...");
  oledDisplay.display();
  
  Mcu.begin();
  
  rssi=0;

  RadioEvents.RxDone = OnRxDone;
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                             LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                             LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                             0, true, 0, 0, LORA_IQ_INVERSION_ON, true );
  oledDisplay.clear();
  oledDisplay.drawString(0, 0, "Start First RX...");
  oledDisplay.display();                          
}



void loop()
{
  if(lora_idle)
  {
    lora_idle = false;
    Serial.println("into RX mode");
    Radio.Rx(0);
  }
  Radio.IrqProcess( );
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    rssi=rssi;
    rxSize=size;
    memcpy(buffer, payload, size );
    Radio.Sleep( );
    Serial.printf("\r\nreceived packet - (rssi: %d) \"%d\", \"%d\", \"%d\", \"%d\"\r\n",rssi,buffer[0],buffer[1],buffer[2],buffer[3]);
    
    sprintf(str1,"RX: %d %d %d %d",buffer[0],buffer[1],buffer[2],buffer[3]);
    sprintf(str2,"RSSI: %d", rssi);
    oledDisplay.clear();
    oledDisplay.drawString(0, 0, str1);
    oledDisplay.drawString(0,10, str2);
    oledDisplay.display();
    lora_idle = true;
}
