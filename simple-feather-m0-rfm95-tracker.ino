#include "lora_secrets.h"
#include <TinyGPSPlus.h>
#include <lmic.h>
#include <hal/hal.h>
#include <FlashStorage.h>

// From lmic documentation:
// Set this to 1 to enable some basic debug output (using printf) about
// RF settings used during transmission and reception. Set to 2 to
// enable more verbose output. Make sure that printf is actually
// configured (e.g. on AVR it is not by default), otherwise using it can
// cause crashing.
#define LMIC_DEBUG_LEVEL 1

// Any runtime assertion failures are printed to this serial port (or
// any other Print object). If this is unset, any failures just silently
// halt execution.
#define LMIC_FAILURE_TO Serial

/* Using USE_SAVED_KEYS prevents rejoining every time the device is powered up*/
#define USE_SAVED_KEYS

/* FRAME_CNT_SAVE_INT is the interval to save frame count. To not wear down flash
we set this to something large (i.e. 1000 with a 60 second transmit interval will only store frame count
every 1000 minutes). Everytime we power the device on we add this number to the stored frame count since it 
will be higher than the current frame count we did not store. The frame count is fine if it jumps up but packets
will get rejected if frame count is lower than a previously transmitted packet. */
#define FRAME_CNT_SAVE_INT 1000

/* DUTY_CYLE defines how often to transmit in seconds. Make sure to use airtime calculator
to determine legally accepted intervals. See (https://avbentem.github.io/airtime-calculator/ttn/us915/18)*/
#define DUTY_CYLE 60

// Storage for packet that is transmitted
uint8_t data_frame[11];

// struct to store all keys after a join
typedef struct {
    u4_t netid;
    devaddr_t devaddr;
    u1_t nwkKey[16];
    u1_t artKey[16];
} lmic_keys_storage_t;

// Flash Storage
FlashStorage(frame_cnt, uint32_t); // stores frame counter
FlashStorage(stored_lmic_keys, lmic_keys_storage_t); // stores all network keys after join

// GPS
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;

uint32_t timer = millis();

// Pin mapping Feather M0
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, 11},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};

// Get lora keys from lora_secrets.h
static const u1_t PROGMEM APPEUI[8]= SECRET_APP_EUI;
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}
static const u1_t PROGMEM DEVEUI[8]= SECRET_DEV_EUI;
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}
static const u1_t PROGMEM APPKEY[16] = SECRET_APP_KEY;
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

volatile int tx_ready = 0;

// -------- LoRa Event 
void onEvent (ev_t ev) { 
  Serial.print(os_getTime());
  Serial.print(": ");
  switch(ev) {
  case EV_SCAN_TIMEOUT:
    Serial.println(F("EV_SCAN_TIMEOUT"));
    break;
  case EV_BEACON_FOUND:
    Serial.println(F("EV_BEACON_FOUND"));
    break;
  case EV_BEACON_MISSED:
    Serial.println(F("EV_BEACON_MISSED"));
    break;
  case EV_BEACON_TRACKED:
    Serial.println(F("EV_BEACON_TRACKED"));
    break;
  case EV_JOINING:
    Serial.println(F("EV_JOINING"));
    break;
  case EV_JOINED:
    Serial.println(F("EV_JOINED"));
    print_network_info();
    lmic_keys_storage_t keys;
    LMIC_getSessionKeys(&keys.netid, &keys.devaddr, keys.nwkKey, keys.artKey);
    stored_lmic_keys.write(keys);
    // Disable link check validation (automatically enabled
    // during join, but not supported by TTN at this time).
    LMIC_setLinkCheckMode(0);
    break;
  case EV_RFU1:
    Serial.println(F("EV_RFU1"));
    break;
  case EV_JOIN_FAILED:
    Serial.println(F("EV_JOIN_FAILED"));
    break;
  case EV_REJOIN_FAILED:
    Serial.println(F("EV_REJOIN_FAILED"));
    break;
    break;
  case EV_TXCOMPLETE:
    Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
    if (LMIC.txrxFlags & TXRX_ACK)
      Serial.println(F("Received ack"));
    if (LMIC.dataLen) {
      Serial.println(F("Received "));
      Serial.println(LMIC.dataLen);
      Serial.println(F(" bytes of payload"));
      for (int i = 0;i<LMIC.dataLen;i++) { 
        Serial.println(LMIC.frame[i+ LMIC.dataBeg],HEX);
      }
    }
    if (LMIC.seqnoUp % FRAME_CNT_SAVE_INT == 0)
    {
      Serial.print("Frame cnt increment by ");
      Serial.print(FRAME_CNT_SAVE_INT);
      Serial.println(" writing to flash");
      frame_cnt.write(LMIC.seqnoUp);
    }
    else
    {
      Serial.print("Frame cnt curerntly is: ");
      Serial.print(LMIC.seqnoUp);
      Serial.print(" stored cnt is: ");
      Serial.println(frame_cnt.read());
    }
    tx_ready = 1;
    break;
  case EV_LOST_TSYNC:
    Serial.println(F("EV_LOST_TSYNC"));
    break;
  case EV_RESET:
    Serial.println(F("EV_RESET"));
    break;
  case EV_RXCOMPLETE:
    // data received in ping slot
    Serial.println(F("EV_RXCOMPLETE"));
    break;
  case EV_LINK_DEAD:
    Serial.println(F("EV_LINK_DEAD"));
    break;
  case EV_LINK_ALIVE:
    Serial.println(F("EV_LINK_ALIVE"));
    break;
  default:
    Serial.println(F("Unknown event"));
    Serial.println((int)ev);
    break;
  }
}

void setup(){
  Serial.begin(115200);
  Serial1.begin(GPSBaud);

  delay(5000);
  
  // Init lmic stuff
  os_init();
  LMIC_reset();

  // If we want to use saved keys (you should want to)
  #ifdef USE_SAVED_KEYS
  {
    Serial.println("Using stored keys");
    lmic_keys_storage_t keys;
    keys = stored_lmic_keys.read();
    if (keys.netid != 0)
    {
      LMIC_setSession (keys.netid, keys.devaddr, keys.nwkKey, keys.artKey);
      LMIC.seqnoUp = frame_cnt.read() + FRAME_CNT_SAVE_INT;
      frame_cnt.write(LMIC.seqnoUp);
      print_network_info();
    }
    else
    {
      Serial.println("Failed to get saved network keys. Hopefully you just reprogrammed");
      Serial.println("Rejoining network");
    }
  }
  #endif

  LMIC_setClockError(MAX_CLOCK_ERROR * 50 / 100); // important for uplinks
  LMIC_setLinkCheckMode(0);
  LMIC_setDrTxpow(DR_SF9,14);
  LMIC_selectSubBand(1);

}

void loop() { 

#ifdef DEBUG_GPS
  while(true)
  {
    int a = Serial1.read();
    if (a > -1)
    {
      Serial.print((char)a);
    }
    
    delay(1);
  }
#endif

  while (Serial1.available() > 0)
  
  if (gps.encode(Serial1.read()))
  {

    if (gps.location.isValid()) {

    Serial.print(gps.altitude.meters(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
    Serial.print(F(","));
    Serial.println(gps.location.isValid(), 6);
        
    
    if (LMIC.opmode & OP_TXRXPEND) { 
      Serial.println(F("OP_TXRXPEND, not sending"));
    } 
    else {
      // Generate packet to send
      tx_ready = 0;    
      uint32_t lat = (gps.location.lat() + 90.0 )* 10000000; // Convert to positive integer
      uint32_t lon = (gps.location.lng() +180.0) * 10000000;
      uint32_t altitude = gps.altitude.meters() + 1000; // offset by 1000 since storing in unsigned int and want some negative values 
      uint8_t hdop = gps.hdop.value()/10; 

      Serial.println(lat);
      Serial.println(lon);
      Serial.println(altitude);
      Serial.println(hdop);

      // 32 bits for latitude
      data_frame[0] = lat;
      data_frame[1] = lat >> 8;
      data_frame[2] = lat >> 16;
      data_frame[3] = lat >> 24;

      // 32 bits for longitude
      data_frame[4] = lon;
      data_frame[5] = lon >> 8;
      data_frame[6] = lon >> 16;
      data_frame[7] = lon >> 24;

      // 16 bits for altitude
      data_frame[8] = altitude;
      data_frame[9] = altitude >> 8;

      // 8 bits for HDOP
      data_frame[10] = hdop;

      for (int i = 0; i < sizeof(data_frame); i++)
      {
          printHex2(data_frame[i]);
      }
      Serial.println("");
   
      LMIC_setTxData2(1, (uint8_t*) data_frame, sizeof(data_frame), 0);
   
      Serial.println(F("Packet queued"));
      while(tx_ready==0) {
        yield();
        os_runloop_once();
      };

      Serial.print(F("Waiting "));

      for (int i = 0;i<DUTY_CYLE;i++) { 
        Serial.print(F("."));
        delay(1000); 
      }
      Serial.println(" done.");

     }
    } else {
      Serial.println("no GPS fix");
      Serial.print("Number of current sats: ");
      Serial.println(gps.satellites.value());
      delay(1000);
    }
 }
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void print_network_info()
{
  u4_t netid = 0;
  devaddr_t devaddr = 0;
  u1_t nwkKey[16];
  u1_t artKey[16];
  LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
  Serial.print("netid: ");
  Serial.println(netid, DEC);
  Serial.print("devaddr: ");
  Serial.println(devaddr, DEC);
  Serial.print("AppSKey: ");
  for (size_t i=0; i<sizeof(artKey); ++i) {
    if (i != 0)
      Serial.print("-");
    printHex2(artKey[i]);
  }
  Serial.println("");
  Serial.print("NwkSKey: ");
  for (size_t i=0; i<sizeof(nwkKey); ++i) {
          if (i != 0)
                  Serial.print("-");
          printHex2(nwkKey[i]);
  }
  Serial.println();
}
