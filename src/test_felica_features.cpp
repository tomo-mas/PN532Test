/**************************************************************************/
/*!
    This example will attempt to connect to an FeliCa
    card or tag and retrieve some basic information about it
    that can be used to determine what type of card it is.

    Note that you need the baud rate to be 115200 because we need to print
    out the data and read from the card at the same time!

    To enable debug message, define DEBUG in PN532/PN532_debug.h

 */
/**************************************************************************/
#include <Arduino.h>

#if 1
  #include <SPI.h>
  #include <PN532_SPI.h>
  #include <PN532.h>

PN532_SPI pn532spi(SPI, 10);
PN532 nfc(pn532spi);
#elif 0
  #include <PN532_HSU.h>
  #include <PN532.h>

PN532_HSU pn532hsu(Serial1);
PN532 nfc(pn532hsu);
#else
  #include <Wire.h>
  #include <PN532_I2C.h>
  #include <PN532.h>

PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);
#endif

#include <PN532_debug.h>

uint8_t        _prevIDm[8];
unsigned long  _prevTime;

void printHex(uint8_t a);
void printHex(uint16_t a);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Hello!");

  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata)
  {
    Serial.print("Didn't find PN53x board");
    while (1) {delay(10);};      // halt
  }

  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata >> 24) & 0xFF, HEX);
  Serial.print("Firmware ver. "); Serial.print((versiondata >> 16) & 0xFF, DEC);
  Serial.print('.'); Serial.println((versiondata >> 8) & 0xFF, DEC);

  // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0xFF);
  nfc.SAMConfig();

  memset(_prevIDm, 0, 8);
}

void loop(void)
{
  uint8_t ret;
  uint16_t systemCode = 0xFFFF;
  uint8_t requestCode = 0x01;       // System Code request
  uint8_t idm[8];
  uint8_t pmm[8];
  uint16_t systemCodeResponse;

  // Wait for an FeliCa type cards.
  // When one is found, some basic information such as IDm, PMm, and System Code are retrieved.
  Serial.print("Waiting for an FeliCa card...  ");
  ret = nfc.felica_Polling(systemCode, requestCode, idm, pmm, &systemCodeResponse, 5000);

  if (ret != 1)
  {
    Serial.println("Could not find a card");
    delay(500);
    return;
  }

  if ( memcmp(idm, _prevIDm, 8) == 0 ) {
    if ( (millis() - _prevTime) < 3000 ) {
      Serial.println("Same card");
      delay(500);
      return;
    }
  }

  Serial.println("Found a card!");
  Serial.print("  IDm: ");
  nfc.PrintHex(idm, 8);
  Serial.print("  PMm: ");
  nfc.PrintHex(pmm, 8);
  Serial.print("  System Code: ");
  Serial.print(systemCodeResponse, HEX);
  Serial.print("\n");

  memcpy(_prevIDm, idm, 8);
  _prevTime = millis();

  Serial.print("Request Service command -> ");
  uint16_t nodeCodeList[3] = {0x0000, 0x1000, 0xFFFF};
  uint16_t keyVersions[3];
  ret = nfc.felica_RequestService(3, nodeCodeList, keyVersions);
  if (ret != 1)
  {
    Serial.println("error");
  } else {
    Serial.println("OK!");
    for(int i=0; i<3; i++ ) {
      Serial.print("  Node Code: "); printHex(nodeCodeList[i]);
      Serial.print(" -> Key Version: "); printHex(keyVersions[i]);
      Serial.println("");
    }
  }

  Serial.print("Read Without Encryption command -> ");
  uint8_t blockData[3][16];
  uint16_t serviceCodeList[1] = {0x000B};
  uint16_t blockList[3] = {0x8000, 0x8001, 0x8002};
  ret = nfc.felica_ReadWithoutEncryption(1, serviceCodeList, 3, blockList, blockData);
  if (ret != 1)
  {
    Serial.println("error");
  } else {
    Serial.println("OK!");
    for(int i=0; i<3; i++ ) {
      Serial.print("  Block no. "); Serial.print(i, DEC); Serial.print(": ");
      nfc.PrintHex(blockData[i], 16);
    }
  }

  Serial.print("Request Response command -> ");
  uint8_t mode;
  ret = nfc.felica_RequestResponse(&mode);
  if (ret != 1)
  {
    Serial.println("error");
  } else {
    Serial.println("OK!");
    Serial.print("  mode: "); Serial.println(mode, DEC);
  }

  Serial.print("Request System Code command -> ");
  uint8_t numSystemCode;
  uint16_t systemCodeList[16];
  ret = nfc.felica_RequestSystemCode(&numSystemCode, systemCodeList);
  if (ret != 1)
  {
    Serial.println("error");
  } else {
    Serial.println("OK!");
    for(int i=0; i< numSystemCode; i++) {
      Serial.print("  System code: ");  printHex(systemCodeList[i]); Serial.println("");
    }
  }

  // Wait 1 second before continuing
  Serial.println("Card access completed!\n");
  delay(1000);
}

void printHex(uint8_t a) {
    Serial.print((a >> 4)&0x0F, HEX);
    Serial.print(a&0x0F, HEX);
}

void printHex(uint16_t a) {
    Serial.print((a >> 12)&0x0F, HEX);
    Serial.print((a >> 8)&0x0F, HEX);
    Serial.print((a >> 4)&0x0F, HEX);
    Serial.print((a)&0x0F, HEX);
}
