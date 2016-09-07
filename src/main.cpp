#include <Arduino.h>
#include <Wire.h>

#define PN532_PREAMBLE                (0x00)
#define PN532_STARTCODE1              (0x00)
#define PN532_STARTCODE2              (0xFF)
#define PN532_POSTAMBLE               (0x00)
#define PN532_HOSTTOPN532             (0xD4)
#define PN532_PN532TOHOST             (0xD5)
#define PN532_ACK_WAIT_TIME           (10)  // ms, timeout of waiting for ACK
#define PN532_I2C_ADDRESS             (0x48 >> 1)
#define TIMEOUT                       (1000)


const uint8_t PN532_ACK[] = {0, 0, 0xFF, 0, 0xFF, 0};
uint8_t buf[256];

void printHex(uint8_t d);
void printHex(uint8_t d[], uint8_t len);
int8_t sendCommand(const uint8_t *cmd, uint8_t cLen);


void printHex(uint8_t d) {
  Serial.print((d >> 4)&0x0F, HEX);
  Serial.print((d)&0x0F, HEX);
}

void printHex(uint8_t d[], uint8_t len) {
  for(uint8_t i=0; i<len; i++) {
    printHex(d[i]);
  }
}

int8_t sendCommand(const uint8_t *cmd, uint8_t cLen)
{
    uint8_t i, k=0;

    buf[k++] = PN532_PREAMBLE;
    buf[k++] = PN532_STARTCODE1;
    buf[k++] = PN532_STARTCODE2;

    uint8_t length = cLen + 1;   // length of data field: TFI + DATA
    buf[k++] = length;
    buf[k++] = ~length + 1;                 // checksum of length

    buf[k++] = PN532_HOSTTOPN532;
    uint8_t sum = PN532_HOSTTOPN532;    // sum of TFI + DATA

    for (i = 0; i < cLen; i++) {
      buf[k++] = cmd[i];
      sum += cmd[i];
    }

    uint8_t checksum = ~sum + 1;            // checksum of TFI + DATA
    buf[k++] = checksum;
    buf[k++] = PN532_POSTAMBLE;

    Serial.print("Send: ");
    printHex(buf, k);
    Serial.print("\n");

    Wire.beginTransmission(PN532_I2C_ADDRESS);
    for (i = 0; i < k; i++) {
      Wire.write(buf[i]);
    }
    Wire.endTransmission();

    // wait ACK
    delay(PN532_ACK_WAIT_TIME);
    Wire.requestFrom(PN532_I2C_ADDRESS,  1 + 6); // read Status + ACK(6)
    buf[0] = Wire.read();
    if ( (buf[0] & 0x01) != 1 ) {
      // error
      Serial.print("ACK wait timeout");
      return -1;
    }

    // read ACK
    for (i = 0; i < 6; i++) {
        buf[i+1] = Wire.read();
    }
    Serial.print("Receive ACK: ");
    printHex(buf, 7);
    Serial.print("\n");

    // wait response
    delay(1);
    unsigned long start = millis();
    do {
      Wire.requestFrom(PN532_I2C_ADDRESS,  1); // read Status + ACK(6)
      buf[0] = Wire.read();
      if ( (buf[0] & 0x01) == 1 ) {
        break;
      }
      if ( (millis() - start) > TIMEOUT ) {
        Serial.print("response wait timeout");
        return -2;
      }
    } while(1);

    // read response (first 4 byte)
    for (i = 1; i < 5; i++) {
        buf[i] = Wire.read();
    }
    uint8_t rLen = buf[4];
    for (i = 5; i < rLen + 8; i++) {
        buf[i] = Wire.read();
    }

    Serial.print("Sesponse: ");
    printHex(buf, rLen + 8);
    Serial.print("\n");
    return 0;
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Hello!");

  // begin
  Wire.begin();

  // wakeup
  Wire.beginTransmission(PN532_I2C_ADDRESS); // I2C start
  delay(20);
  Wire.endTransmission();                    // I2C end

  uint8_t cmd[] = {0x02};
  sendCommand(cmd, 1);

}

void loop(void)
{

}
