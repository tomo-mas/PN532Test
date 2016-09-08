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

#define T_UNKNOWN                   (0)
#define T_ACK                       (1)
#define T_DATA                      (2)

const uint8_t PN532_PRE_START[] = {0, 0, 0xFF};
const uint8_t PN532_ACK[] = {0, 0xFF, 0};
uint8_t buf[35];

void printHex(uint8_t d);
void printHex(uint8_t d[], uint8_t len);
int8_t sendData(const uint8_t *cmd, uint8_t cLen);


void printHex(uint8_t d) {
  Serial.print((d >> 4) & 0x0F, HEX);
  Serial.print((d) & 0x0F, HEX);
}

void printHex(uint8_t d[], uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    printHex(d[i]);
  }
}

uint8_t parse (const uint8_t *cmd, const uint8_t len, uint8_t *type, uint8_t *dataLen, uint8_t *data) {
  if ( (buf[0] & 0x01) != 1 ) {
    Serial.println("Error response");
    return -1;
  }

  if ( len < 7 ) {
    *type = T_UNKNOWN;
    return 1;
  }

  // check response frame
  if ( memcmp( &buf[1], PN532_PRE_START, 3) != 0 ) {
    Serial.println("Wrong frame");
    return -2;
  }

  if ( memcmp( &buf[4], PN532_ACK, 3) == 0 ) {
    Serial.println("ACK received");
    *type = T_ACK;
    return 1;
  }

  if ( (buf[4] + buf[5]) & 0xFF != 0 ) {
    Serial.println(buf[4] + buf[5]);
    Serial.println( (buf[4] + buf[5]) & 0xFF);
    Serial.println("Wrong Length");
    return -1;
  }

  Serial.print("LEN="); Serial.println(buf[4]);
  *type = T_DATA;
  *dataLen = buf[4];
  memcpy(data, &buf[7], *dataLen);
  return 1;
}

int8_t sendData(const uint8_t *cmd, uint8_t cLen)
{
  uint8_t i, k = 0;

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

  Serial.print("Send: ");  printHex(buf, k); Serial.print("\n");
  Wire.beginTransmission(PN532_I2C_ADDRESS);
  for (i = 0; i < k; i++) {
    Wire.write(buf[i]);
  }
  Wire.endTransmission();

  // wait ACK
  delay(10);

  // read ACK
  Wire.requestFrom(PN532_I2C_ADDRESS,  sizeof(buf)); // read Status + ACK(6)
  Serial.print("Wire.available()="); Serial.println(Wire.available());
  uint8_t bufIndex = 0;
  while (Wire.available()) {
    buf[bufIndex++] = Wire.read();
  }

  Serial.print("Received data: "); printHex(buf, bufIndex); Serial.print("\n");

  uint8_t type = T_UNKNOWN;
  uint8_t dataLen = 0;
  uint8_t dataBuf[32];
  if ( parse (buf, bufIndex, &type, &dataLen, dataBuf) != 1 ) {
    return -1;
  }
  Serial.print("type="); Serial.println(type);
  Serial.print("len="); Serial.println(bufIndex);
  Serial.print("dataLen="); Serial.println(dataLen);

  // wait response
  unsigned long start = millis();
  unsigned long el = (millis() - start);
  while ( 1 ) {
    Wire.requestFrom(PN532_I2C_ADDRESS, sizeof(buf));
    Serial.print("Wire.available()="); Serial.println(Wire.available());
    bufIndex = 0;
    while (Wire.available()) {
      buf[bufIndex++] = Wire.read();
    }

    Serial.print("Response: "); printHex(buf, bufIndex);  Serial.print("\n");

    printHex(buf, bufIndex); Serial.print("\n");
    if ( parse (buf, bufIndex, &type, &dataLen, dataBuf) != 1 ) {
      return -1;
    }
    Serial.print("data: "); printHex(dataBuf, dataLen);  Serial.print("\n");
    Serial.print("type="); Serial.println(type);
    Serial.print("len="); Serial.println(bufIndex);
    Serial.print("dataLen="); Serial.println(dataLen);
    delay(10);
    Serial.print("elapsed: "); Serial.println(el);
    if ( el > TIMEOUT ) {
      Serial.print("response wait timeout");
      return -2;
    }
    break;
  }

  return 0;
}

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Hello!");

  delay(500);
  // begin
  Wire.begin();

  // wakeup
  Wire.beginTransmission(PN532_I2C_ADDRESS); // I2C start
  delay(20);
  Wire.endTransmission();                    // I2C end

  uint8_t cmd[] = {0x02};
  sendData(cmd, 1);
}

void loop(void)
{
  /*
  Serial.println("D1");
  uint8_t cmd[] = {0x02};
  sendData(cmd, 1);
  delay(1000);
  Serial.println("D3");
  */
  delay(1000);
}
