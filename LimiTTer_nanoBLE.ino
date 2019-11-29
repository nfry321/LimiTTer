/*
   TODO
   rename
   document
   wiring connections
   Bluetooth
   rgb led feedback
   connect battery
   sleep code

  NFC Communication with the Solutions Cubed, LLC BM019
  and an Arduino Uno.  The BM019 is a module that
  carries ST Micro's CR95HF, a serial to NFC converter.

  Wiring:
  Arduino          BM019
  IRQ: Pin 9       DIN: pin 2
  SS: pin 10       SS: pin 3
  MOSI: pin 11     MOSI: pin 5
  MISO: pin 12     MISO: pin4
  SCK: pin 13      SCK: pin 6

*/

// the sensor communicates using SPI, so include the library:
#include <SPI.h>
#include <ArduinoBLE.h>

#define MAX_NFC_READTRIES 10 // Amount of tries for every nfc block-scan

BLEService outputService("0x181F"); //BLE Cont glucose service
BLECharacteristic cgm_measurement("LimiTTer", BLERead | BLENotify,16); //CGM Measurement Characteristic "0x2AA7"
const int pkglen = 16;
char MyString[16];

//for glucose reading
int sensorMinutesElapse;
float lastGlucose;
byte FirstRun = 1;
int noDiffCount = 0;
float trend[16];
byte batteryLow;
int batteryPcnt;
long batteryMv;

const int SSPin = 10;  // Slave Select pin
const int IRQPin = 9;  // Sends wake-up pulse
byte TXBuffer[40];    // transmit buffer
byte RXBuffer[40];    // receive buffer
byte NFCReady = 0;  // used to track NFC state
byte Memory_Block = 0; // keep track of memory we write to
byte Data = 0; // keep track of memory we read from


void setup() {
  pinMode(IRQPin, OUTPUT);
  digitalWrite(IRQPin, HIGH); // Wake up pulse
  pinMode(SSPin, OUTPUT);
  digitalWrite(SSPin, HIGH);

  Serial.begin(9600);
  SPI.begin();
  //SPI.setDataMode(SPI_MODE0); //default values and causing issues with SPI.h  - think these can be set with an spisettings function if required
  //SPI.setBitOrder(MSBFIRST);
  //SPI.setClockDivider(SPI_CLOCK_DIV32);

  //start BLE
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  // set advertised local name and service UUID:
  BLE.setLocalName("LimiTTer");
  BLE.setAdvertisedService(outputService);
  outputService.addCharacteristic(cgm_measurement);
  BLE.addService(outputService);
  // start advertising
  BLE.advertise();
  Serial.println("BLE advertising");

  // The CR95HF requires a wakeup pulse on its IRQ_IN pin
  // before it will select UART or SPI mode.  The IRQ_IN pin
  // is also the UART RX pin for DIN on the BM019 board.

  delay(10);                      // send a wake up
  digitalWrite(IRQPin, LOW);      // pulse to put the
  delayMicroseconds(100);         // BM019 into SPI
  digitalWrite(IRQPin, HIGH);     // mode
  delay(10);
}


/* SetProtocol_Command programs the CR95HF for
  ISO/IEC 15693 operation.

  This requires three steps.
  1. send command
  2. poll to see if CR95HF has data
  3. read the response

  If the correct response is received the serial monitor is used
  to display successful programming.
*/
void SetProtocol_Command()
{

  // step 1 send the command
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x02);  // Set protocol command
  SPI.transfer(0x02);  // length of data to follow
  SPI.transfer(0x01);  // code for ISO/IEC 15693
  SPI.transfer(0x0D);  // Wait for SOF, 10% modulation, append CRC
  digitalWrite(SSPin, HIGH);
  delay(1);

  // step 2, poll for data ready

  digitalWrite(SSPin, LOW);
  while (RXBuffer[0] != 8)
  {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
  }
  digitalWrite(SSPin, HIGH);
  delay(1);

  // step 3, read the data
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  digitalWrite(SSPin, HIGH);

  if ((RXBuffer[0] == 0) & (RXBuffer[1] == 0))  // is response code good?
  {
    Serial.println("Protocol Set Command OK");
    NFCReady = 1; // NFC is ready
  }
  else
  {
    Serial.println("Protocol Set Command FAIL");
    NFCReady = 0; // NFC not ready
  }
}

/* Inventory_Command chekcs to see if an RF
  tag is in range of the BM019.

  This requires three steps.
  1. send command
  2. poll to see if CR95HF has data
  3. read the response

  If the correct response is received the serial monitor is used
  to display the the RF tag's universal ID.
*/
void Inventory_Command()
{
  byte i = 0;

  // step 1 send the command
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
  SPI.transfer(0x04);  // Send Receive CR95HF command
  SPI.transfer(0x03);  // length of data that follows is 0
  SPI.transfer(0x26);  // request Flags byte
  SPI.transfer(0x01);  // Inventory Command for ISO/IEC 15693
  SPI.transfer(0x00);  // mask length for inventory command
  digitalWrite(SSPin, HIGH);
  delay(1);

  // step 2, poll for data ready
  // data is ready when a read byte
  // has bit 3 set (ex:  B'0000 1000')

  digitalWrite(SSPin, LOW);
  while (RXBuffer[0] != 8)
  {
    RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
    RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
  }
  digitalWrite(SSPin, HIGH);
  delay(1);


  // step 3, read the data
  digitalWrite(SSPin, LOW);
  SPI.transfer(0x02);   // SPI control byte for read
  RXBuffer[0] = SPI.transfer(0);  // response code
  RXBuffer[1] = SPI.transfer(0);  // length of data
  for (i = 0; i < RXBuffer[1]; i++)
    RXBuffer[i + 2] = SPI.transfer(0); // data
  digitalWrite(SSPin, HIGH);
  delay(1);

  if (RXBuffer[0] == 128)  // is response code good?
  {
    Serial.println("Sensor in range ... OK");
    NFCReady = 2;
  }
  else
  {
    Serial.println("Sensor out of range ... FAIL");
    NFCReady = 0;
  }
}

/* Read Memory reads data from a block of memory.
  This code assumes the RF tag has less than 256 blocks
  of memory and that the block size is 4 bytes.  Data
  read from is displayed via the serial monitor.

  This requires three steps.
  1. send command
  2. poll to see if CR95HF has data
  3. read the response
*/
float Read_Memory() {

  byte oneBlock[8];
  String hexPointer = "";
  String trendValues = "";
  String hexMinutes = "";
  String elapsedMinutes = "";
  float trendOneGlucose;
  float trendTwoGlucose;
  float currentGlucose;
  float shownGlucose;
  float averageGlucose = 0;
  int glucosePointer;
  int validTrendCounter = 0;
  float validTrend[16];
  byte readError = 0;
  int readTry;

  for ( int b = 3; b < 16; b++) {
    readTry = 0;
    do {
      readError = 0;
      digitalWrite(SSPin, LOW);
      SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
      SPI.transfer(0x04);  // Send Receive CR95HF command
      SPI.transfer(0x03);  // length of data that follows
      SPI.transfer(0x02);  // request Flags byte
      SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
      SPI.transfer(b);  // memory block address
      digitalWrite(SSPin, HIGH);
      delay(1);

      digitalWrite(SSPin, LOW);
      while (RXBuffer[0] != 8)
      {
        RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
        RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
      }
      digitalWrite(SSPin, HIGH);
      delay(1);

      digitalWrite(SSPin, LOW);
      SPI.transfer(0x02);   // SPI control byte for read
      RXBuffer[0] = SPI.transfer(0);  // response code
      RXBuffer[1] = SPI.transfer(0);  // length of data
      for (byte i = 0; i < RXBuffer[1]; i++)
        RXBuffer[i + 2] = SPI.transfer(0); // data
      if (RXBuffer[0] != 128)
        readError = 1;
      digitalWrite(SSPin, HIGH);
      delay(1);

      for (int i = 0; i < 8; i++)
        oneBlock[i] = RXBuffer[i + 3];

      char str[24];
      unsigned char * pin = oneBlock;
      const char * hex = "0123456789ABCDEF";
      char * pout = str;
      for (; pin < oneBlock + 8; pout += 2, pin++) {
        pout[0] = hex[(*pin >> 4) & 0xF];
        pout[1] = hex[ *pin     & 0xF];
      }
      pout[0] = 0;
      if (!readError)       // is response code good?
      {
        Serial.println(str);
        trendValues += str;
      }
      readTry++;
    } while ( (readError) && (readTry < MAX_NFC_READTRIES) );

  }
  readTry = 0;
  do {
    readError = 0;
    digitalWrite(SSPin, LOW);
    SPI.transfer(0x00);  // SPI control byte to send command to CR95HF
    SPI.transfer(0x04);  // Send Receive CR95HF command
    SPI.transfer(0x03);  // length of data that follows
    SPI.transfer(0x02);  // request Flags byte
    SPI.transfer(0x20);  // Read Single Block command for ISO/IEC 15693
    SPI.transfer(39);  // memory block address
    digitalWrite(SSPin, HIGH);
    delay(1);

    digitalWrite(SSPin, LOW);
    while (RXBuffer[0] != 8)
    {
      RXBuffer[0] = SPI.transfer(0x03);  // Write 3 until
      RXBuffer[0] = RXBuffer[0] & 0x08;  // bit 3 is set
    }
    digitalWrite(SSPin, HIGH);
    delay(1);

    digitalWrite(SSPin, LOW);
    SPI.transfer(0x02);   // SPI control byte for read
    RXBuffer[0] = SPI.transfer(0);  // response code
    RXBuffer[1] = SPI.transfer(0);  // length of data
    for (byte i = 0; i < RXBuffer[1]; i++)
      RXBuffer[i + 2] = SPI.transfer(0); // data
    if (RXBuffer[0] != 128)
      readError = 1;
    digitalWrite(SSPin, HIGH);
    delay(1);

    for (int i = 0; i < 8; i++)
      oneBlock[i] = RXBuffer[i + 3];

    char str[24];
    unsigned char * pin = oneBlock;
    const char * hex = "0123456789ABCDEF";
    char * pout = str;
    for (; pin < oneBlock + 8; pout += 2, pin++) {
      pout[0] = hex[(*pin >> 4) & 0xF];
      pout[1] = hex[ *pin     & 0xF];
    }
    pout[0] = 0;
    if (!readError)
      elapsedMinutes += str;
    readTry++;
  } while ( (readError) && (readTry < MAX_NFC_READTRIES) );

  if (!readError)
  {
    hexMinutes = elapsedMinutes.substring(10, 12) + elapsedMinutes.substring(8, 10);
    hexPointer = trendValues.substring(4, 6);
    sensorMinutesElapse = strtoul(hexMinutes.c_str(), NULL, 16);
    glucosePointer = strtoul(hexPointer.c_str(), NULL, 16);

    Serial.println("");
    Serial.print("Glucose pointer: ");
    Serial.print(glucosePointer);
    Serial.println("");

    int ii = 0;
    for (int i = 8; i <= 200; i += 12) {
      if (glucosePointer == ii)
      {
        if (glucosePointer == 0)
        {
          String trendNow = trendValues.substring(190, 192) + trendValues.substring(188, 190);
          String trendOne = trendValues.substring(178, 180) + trendValues.substring(176, 178);
          String trendTwo = trendValues.substring(166, 168) + trendValues.substring(164, 166);
          currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL , 16));
          trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL , 16));
          trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL , 16));

          if (FirstRun == 1)
            lastGlucose = currentGlucose;

          if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
          {
            if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
              currentGlucose = trendTwoGlucose;
            else
              currentGlucose = trendOneGlucose;
          }
        }
        else if (glucosePointer == 1)
        {
          String trendNow = trendValues.substring(i - 10, i - 8) + trendValues.substring(i - 12, i - 10);
          String trendOne = trendValues.substring(190, 192) + trendValues.substring(188, 190);
          String trendTwo = trendValues.substring(178, 180) + trendValues.substring(176, 178);
          currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL , 16));
          trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL , 16));
          trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL , 16));

          if (FirstRun == 1)
            lastGlucose = currentGlucose;

          if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
          {
            if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
              currentGlucose = trendTwoGlucose;
            else
              currentGlucose = trendOneGlucose;
          }
        }
        else
        {
          String trendNow = trendValues.substring(i - 10, i - 8) + trendValues.substring(i - 12, i - 10);
          String trendOne = trendValues.substring(i - 22, i - 20) + trendValues.substring(i - 24, i - 22);
          String trendTwo = trendValues.substring(i - 34, i - 32) + trendValues.substring(i - 36, i - 34);
          currentGlucose = Glucose_Reading(strtoul(trendNow.c_str(), NULL , 16));
          trendOneGlucose = Glucose_Reading(strtoul(trendOne.c_str(), NULL , 16));
          trendTwoGlucose = Glucose_Reading(strtoul(trendTwo.c_str(), NULL , 16));


          if (FirstRun == 1)
            lastGlucose = currentGlucose;

          if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
          {
            if (((lastGlucose - trendOneGlucose) > 50) || ((trendOneGlucose - lastGlucose) > 50))
              currentGlucose = trendTwoGlucose;
            else
              currentGlucose = trendOneGlucose;
          }
        }
      }

      ii++;
    }

    for (int i = 8, j = 0; i < 200; i += 12, j++) {
      String t = trendValues.substring(i + 2, i + 4) + trendValues.substring(i, i + 2);
      trend[j] = Glucose_Reading(strtoul(t.c_str(), NULL , 16));
    }

    for (int i = 0; i < 16; i++)
    {
      if (((lastGlucose - trend[i]) > 50) || ((trend[i] - lastGlucose) > 50)) // invalid trend check
        continue;
      else
      {
        validTrend[validTrendCounter] = trend[i];
        validTrendCounter++;
      }
    }

    if (validTrendCounter > 0)
    {
      for (int i = 0; i < validTrendCounter; i++)
        averageGlucose += validTrend[i];

      averageGlucose = averageGlucose / validTrendCounter;

      if (((lastGlucose - currentGlucose) > 50) || ((currentGlucose - lastGlucose) > 50))
        shownGlucose = averageGlucose; // If currentGlucose is still invalid take the average value
      else
        shownGlucose = currentGlucose; // All went well. Take and show the current value
    }
    else
      shownGlucose = currentGlucose; // If all is going wrong, nevertheless take and show a value

    if ((lastGlucose == currentGlucose) && (sensorMinutesElapse > 21000)) // Expired sensor check
      noDiffCount++;

    if (lastGlucose != currentGlucose) // Reset the counter
      noDiffCount = 0;

    if (currentGlucose != 0)
      lastGlucose = currentGlucose;


    NFCReady = 2;
    FirstRun = 0;

    if (noDiffCount > 5)
      return 0;
    else
      return shownGlucose;

  }
  else
  {
    Serial.print("Read Memory Block Command FAIL");
    NFCReady = 0;
    readError = 0;
  }
  return 0;
}

float Glucose_Reading(unsigned int val) {
  int bitmask = 0x0FFF;
  return ((val & bitmask) / 8.5);
}

String Build_Packet(float glucose) {

  // Let's build a String which xDrip accepts as a BTWixel packet

  unsigned long raw = glucose * 1000; // raw_value
  String packet = "";
  packet = String(raw);
  packet += ' ';
  packet += "216";
  packet += ' ';
  packet += String(batteryPcnt);
  packet += ' ';
  packet += String(sensorMinutesElapse);
  Serial.println("");
  Serial.print("Glucose level: ");
  Serial.print(glucose);
  Serial.println("");
  Serial.print("15 minutes-trend: ");
  Serial.println("");
  for (int i = 0; i < 16; i++)
  {
    Serial.print(trend[i]);
    Serial.println("");
  }
  Serial.print("Battery level: ");
  Serial.print(batteryPcnt);
  Serial.print("%");
  Serial.println("");
  Serial.print("Battery mVolts: ");
  Serial.print(batteryMv);
  Serial.print("mV");
  Serial.println("");
  Serial.print("Sensor lifetime: ");
  Serial.print(sensorMinutesElapse);
  Serial.print(" minutes elapsed");
  Serial.println("");
  return packet;
}

void loop() {
  // listen for BLE peripherals to connect:
  Serial.print("Listening for central: ");
  BLEDevice central = BLE.central();
  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());

    // while the central is still connected to peripheral:
    while (central.connected()) {
      if (NFCReady == 0)
      {
        delay(100);
        SetProtocol_Command(); // ISO 15693 settings
      }
      else if (NFCReady == 1)
      {
        delay(100);
        Inventory_Command();
      }
      else
      {
        delay(100);
        String xdripPacket = Build_Packet(Read_Memory());
        xdripPacket.toCharArray(MyString,pkglen);
       cgm_measurement.writeValue(MyString,16);
       Serial.print("packet length: ");
       Serial.println(xdripPacket.length());
      }
      Serial.println(" ");
      delay(2000);
    }
    // when the central disconnects, print it out:
    Serial.print(F("Disconnected from central: "));
    Serial.println(central.address());
  }
}
