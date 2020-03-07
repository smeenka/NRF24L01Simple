/*

Demonstrates simple RX and TX operation.
Any of the Basic_RX examples can be used as a receiver.
Please read through 'NRF24L01Simple.h' for a description of all the methods available in the library.

Radio    Arduino
CE    -> 9
CSN   -> 10 (Hardware SPI SS)
MOSI  -> 11 (Hardware SPI MOSI)
MISO  -> 12 (Hardware SPI MISO)
SCK   -> 13 (Hardware SPI SCK)
IRQ   -> No connection
VCC   -> No more than 3.6 volts
GND   -> GND

*/

#include <NRF24L01Simple.h>

#include <Adafruit_NeoPixel.h>
#define LED_PIN     2
#define LED_COUNT  16

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

#define NRF24L01SIMPLE_DEBUG 1

const static uint8_t TX_ID = 1; // Id of the radio with sender role.
const static uint8_t RX_ID = 2; // Id of the radio we will transmit to.


struct RadioPacket // Any packet up to 32 bytes can be sent.
{
    uint8_t  command;
    uint8_t  fromId;
    uint32_t deltaMicros;
    uint32_t retry;
};
RadioPacket radioData;

NRF24L01Simple radio(10,9);         // Arduino nano


long nextSendMillis = millis() + 1000;
long sendStartMicros = 0;

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(uint8_t i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void handleISR2() {
  radio.checkStatus();
}
void setup()
{
    Serial.begin(115200);
    Serial.println("Testing nrf24L01+ in receive mode");

    strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
    strip.show();            // Turn OFF all pixels ASAP
    strip.setBrightness(50); // Set BRIGHTNESS to about 1/5 (max = 255)
    colorWipe(strip.Color(255,   0,   0)     , 50); // Red
    colorWipe(strip.Color(5,   5,   5)       , 50); // almost off


    //DDR_INPUT(SDA);
    //attachInterrupt(SDA, handleISR2, FALLING);
    
    colorWipe(strip.Color(  50,0,0), 250);   

    
    // By default, 'init' configures the radio to use a 2MBPS bitrate on channel 100 (channels 0-125 are valid).
    // Both the RX and TX radios must have the same bitrate and channel to communicate with each other.
    // You can run the 'ChannelScanner' example to help select the best channel for your environment.
    // You can assign a different bitrate and channel as shown below.
    //   radio.init(RADIO_ID, PINradio_CE, PINradio_CSN, NRF24L01Simple::BITRATE250KBPS, 0)
    //   radio.init(RADIO_ID, PINradio_CE, PINradio_CSN, NRF24L01Simple::BITRATE1MBPS, 75)
    //   radio.init(RADIO_ID, PINradio_CE, PINradio_CSN, NRF24L01Simple::BITRATE2MBPS, 100) // THE DEFAULT
    
    if (!radio.begin(radio.BITRATE1MBPS))
    {
        Serial.println("Cannot communicate with radio");
        while (1); // Wait here forever.
    }
    radio.setChannel(10);
    radio.powerUp(NRF24L01Simple::POWER_LOW, NRF24L01Simple::CHECKSUM_1_BYTE);

    radioData.fromId = RX_ID;
    radioData.command = 'I';

    uint8_t address[5] = { 1, 2, 3, 4, RX_ID };
    radio.setTxPipe(address);
    radio.setRxPipe(1, address);

    Serial.println("INIT Done");
    radio.startRx();
    radio.queueAckData(1, &radioData,10);
    radio.printDetails();

}

void loop()
{
    // By default, 'send' transmits data and waits for an acknowledgement.  If no acknowledgement is received,
    // it will try again up to 16 times.  You can also perform a NO_ACK send that does not request an acknowledgement.
    // The data packet will only be transmitted a single time so there is no guarantee it will be successful.  Any random
    // electromagnetic interference can sporatically cause packets to be lost, so NO_ACK sends are only suited for certain
    // types of situations, such as streaming real-time data where performance is more important than reliability.
    //   radio.send(DESTINATIONradio_ID, &radioData, sizeof(radioData), NRF24L01Simple::NO_ACK)
    //   radio.send(DESTINATIONradio_ID, &radioData, sizeof(radioData), NRF24L01Simple::REQUIRE_ACK) // THE DEFAULT

    long now = millis();

    radio.checkStatus();
    if (radio.hasData() ) {
        radioData.deltaMicros = micros() - sendStartMicros;
        radioData.retry = radio.countRetransmit();
        radioData.command += 1;

        Serial.print("Recevied packet from partner:");
        Serial.print("   retry:");
        Serial.print(radioData.retry);

        ReceivePacket_t* dataP = radio.getData();
        Serial.print(" len:");
        Serial.print(dataP->length);
        Serial.print(" pipe:");
        Serial.println(dataP->pipe);

        // cast received bytes to the data fromat we use
        RadioPacket* packet = (RadioPacket*) dataP->data; 
        Serial.print("command:");
        Serial.print(packet->command);
        Serial.print(" from:");
        Serial.print(packet->fromId);
        Serial.print("len:");
        Serial.print(packet->deltaMicros);
        Serial.print(" retries:");
        Serial.println(packet->retry);

        colorWipe(strip.Color(5,   50,   0)       , 50); 
        radio.startRx();
        radio.queueAckData(1, &radioData,10);
    }


}
