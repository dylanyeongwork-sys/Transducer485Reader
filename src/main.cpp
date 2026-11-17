#include <Arduino.h>
#include <string.h>
#include <U8g2lib.h> //OLED Library
#include <Wire.h> //I2C Library
// =========================
// RS485 Configuration
// =========================

#define RS485_RX_PIN 20
#define RS485_TX_PIN 21
#define RS485_DE_RE  4

#define RS485_BAUD 9600
#define SLAVE_ID    0x01
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,/* clock=*/ 9, /* data=*/ 8, /* reset=*/ U8X8_PIN_NONE);
HardwareSerial RS485Serial(1);


// =========================
// Modbus CRC16
// Polynomial: 0xA001
// =========================

uint16_t modbusCRC(uint8_t *buffer, uint8_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint8_t i = 0; i < length; i++)
    {
        crc ^= buffer[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }

    return crc;
}


// =========================
// Send Modbus Request
// =========================

void sendReadRequest(uint16_t registerAddress)
{
    uint8_t request[8];

    request[0] = SLAVE_ID;
    request[1] = 0x03;                 // Read Holding Registers
    request[2] = registerAddress >> 8;
    request[3] = registerAddress & 0xFF;
    request[4] = 0x00;
    request[5] = 0x01;                 // Read 1 register

    uint16_t crc = modbusCRC(request, 6);

    // Modbus sends CRC low byte first
    request[6] = crc & 0xFF;
    request[7] = crc >> 8;

    // Clear any old data
    while (RS485Serial.available())
        RS485Serial.read();

    // Enable transmitter
    digitalWrite(RS485_DE_RE, HIGH);

    delayMicroseconds(100);

    RS485Serial.write(request, 8);
    RS485Serial.flush();

    delayMicroseconds(100);

    // Enable receiver
    digitalWrite(RS485_DE_RE, LOW);
}


// =========================
// Read Modbus Response
// =========================

bool readResponse(uint8_t *response, uint8_t expectedLength)
{
    uint32_t startTime = millis();

    uint8_t index = 0;

    while (millis() - startTime < 500)
    {
        while (RS485Serial.available())
        {
            response[index++] = RS485Serial.read();

            if (index >= expectedLength)
            {
                // Check CRC
                uint16_t receivedCRC =
                    response[expectedLength - 2] |
                    (response[expectedLength - 1] << 8);

                uint16_t calculatedCRC =
                    modbusCRC(response, expectedLength - 2);

                if (receivedCRC != calculatedCRC)
                {
                    Serial.println("CRC ERROR");
                    return false;
                }

                return true;
            }
        }
    }

    Serial.println("Timeout waiting for response");

    return false;
}


// =========================
// Read One Register
// =========================

bool readRegister(uint16_t registerAddress, int16_t &value)
{
    uint8_t response[7];

    sendReadRequest(registerAddress);

    /*
       Expected response:

       Address
       Function
       Byte count
       Data High
       Data Low
       CRC Low
       CRC High

       01 03 02 XX XX CRC CRC
    */

    if (!readResponse(response, 7))
        return false;

    // Check slave address
    if (response[0] != SLAVE_ID)
    {
        Serial.println("Wrong slave address");
        return false;
    }

    // Check function code
    if (response[1] != 0x03)
    {
        Serial.println("Wrong function code");
        return false;
    }

    // Check byte count
    if (response[2] != 0x02)
    {
        Serial.println("Unexpected byte count");
        return false;
    }

    value = (int16_t)((response[3] << 8) | response[4]);

    return true;
}


// =========================
// Setup
// =========================
int startTime;
void setup()
{
  u8g2.begin();
  Serial.begin(9600);
  delay(1000);

  Serial.println();
  Serial.println("==============================");
  Serial.println("ESP32-C3 RS485 Pressure Reader");
  Serial.println("==============================");

  // MAX485 direction control
  pinMode(RS485_DE_RE, OUTPUT);

  // Start in receive mode
  digitalWrite(RS485_DE_RE, LOW);

  // Start UART1
  RS485Serial.begin(
      RS485_BAUD,
      SERIAL_8N1,
      RS485_RX_PIN,
      RS485_TX_PIN
  );

  startTime = millis();
  Serial.println("RS485 started");
  Serial.println("Baud: 9600");
  Serial.println("Slave: 1");
}

float maxMPA = 0.0f;
unsigned long zeroPressureStart = 0;
bool zeroPressureTiming = false;

float updateMaxMPA(float currentMPA)
{
    // If pressure is zero, start or continue the timer
    if (currentMPA <= 0.0f)
    {
        if (!zeroPressureTiming)
        {
            zeroPressureStart = millis();
            zeroPressureTiming = true;
        }

        // Reset maximum after pressure has been zero for >10 seconds
        if (millis() - zeroPressureStart > 10000UL)
        {
            maxMPA = 0.0f;
        }
    }
    else
    {
        // Pressure is positive: stop the zero-pressure timer
        zeroPressureTiming = false;

        // Update maximum if current pressure is higher
        if (currentMPA > maxMPA)
        {
            maxMPA = currentMPA;
        }
    }

    return maxMPA;
}

// float updateFIR(float currentPressure)
// {
//     static float readings[10] = {0};
//     static int index = 0;
//     static int count = 0;
//     static float sum = 0;

//     // Remove the oldest reading from the sum
//     sum -= readings[index];

//     // Store the new reading
//     readings[index] = currentPressure;

//     // Add the new reading to the sum
//     sum += currentPressure;

//     // Move to the next position
//     index = (index + 1) % 10;

//     // Track how many valid readings we have
//     if (count < 10)
//         count++;

//     // Return average of available readings
//     return sum / count;
// }
float updateFIR(float currentPressure)
{
    static float readings[10] = {0};
    static int index = 0;
    static int count = 0;
    static float sum = 0;

    static float previousForceKN = 0;
    static unsigned long previousMillis = 0;
    static bool firstReading = true;

    const float pistonArea = 49087.385f; // mm², 250 mm diameter

    // Update 10-reading moving average
    sum -= readings[index];
    readings[index] = currentPressure;
    sum += currentPressure;

    index = (index + 1) % 10;

    if (count < 10)
        count++;

    float averageMPa = sum / count;

    // Convert average pressure to force in kN
    float forceKN = averageMPa * pistonArea / 1000.0f;

    unsigned long now = millis();

    if (firstReading)
    {
        previousForceKN = forceKN;
        previousMillis = now;
        firstReading = false;
        return 0.0f;
    }

    unsigned long elapsed = now - previousMillis;

    if (elapsed == 0)
        return 0.0f;

    // Rate of force change in kN/s
    float rateKNs = (forceKN - previousForceKN)
                    / (elapsed / 1000.0f);

    previousForceKN = forceKN;
    previousMillis = now;

    return rateKNs;
}


void drawRightAligned(int rightX, int y, const char* text)
{
    int textWidth = u8g2.getStrWidth(text);
    u8g2.drawStr(rightX - textWidth, y, text);
}
// [BEGIN lopaka generated]
bool HB = false;
static const unsigned char image_cards_hearts_bits[] U8X8_PROGMEM = {0x00,0x66,0xff,0xff,0x7e,0x3c,0x18,0x00};
void drawScreen(bool connection, float MPA, int BAR, int PSI, float freqRate, float FIR, float maxMPA) {
    u8g2.clearBuffer();					// clear the internal memory
    u8g2.setFontMode(1);
    u8g2.setDisplayRotation(U8G2_R0);
    u8g2.setFlipMode(1);

    char MPA_char[15];
    char BAR_char[15];
    char PSI_char[15];
    char freqRate_char[15];
    char FIR_char[15];
    char maxMPA_char[15];
    if(HB==true) {
        u8g2.drawXBMP(114, 18, 8, 8, image_cards_hearts_bits);
    }
    if(connection == false) {
        strcpy(MPA_char,"DISC");
        strcpy(BAR_char,"DISC"); 
        strcpy(PSI_char,"DISC");
        strcpy(maxMPA_char,"DISC");
        strcpy(FIR_char,"DISC");
        strcpy(freqRate_char,"DISC");
    }
    else {
        sprintf(MPA_char,"%.2f",MPA); 
        sprintf(BAR_char,"%1d",BAR); 
        sprintf(PSI_char,"%1d",PSI); 
        sprintf(maxMPA_char,"%.2f",maxMPA);
        sprintf(FIR_char,"%.2f",FIR);
        sprintf(freqRate_char,"%.1f",freqRate); 
    }
  
    u8g2.drawFrame(0, 17, 128, 47);
    u8g2.drawLine(1, 26, 127, 26);
    u8g2.drawLine(67, 27, 67, 62);
    u8g2.setFont(u8g2_font_profont12_tr);
    u8g2.drawStr(7, 8, "Pressure Transducer");
    u8g2.setFont(u8g2_font_profont11_tr);
    u8g2.drawStr(46, 16, "Tester");
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(12, 25, "HPM180 : 250mm Bore");
    
    
    //Pressure Readouts
    u8g2.setFont(u8g2_font_5x8_tr);
    u8g2.drawStr(49, 37, "MPA");
    u8g2.setFont(u8g2_font_profont17_tr);
    drawRightAligned(43, 39, MPA_char);
    // u8g2.drawStr(3, 39, MPA_char);
    
    u8g2.setFont(u8g2_font_5x7_tr);
    u8g2.drawStr(49, 50, "BAR");
    u8g2.setFont(u8g2_font_profont15_tr);
    drawRightAligned(43, 50, BAR_char); 
    // u8g2.drawStr(20, 50, BAR_char);
 
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(49, 60, "PSI");
    u8g2.setFont(u8g2_font_profont10_tr);
    drawRightAligned(43, 60, PSI_char);
    // u8g2.drawStr(23, 61, PSI_char);

    //Data Analytics
    u8g2.setFont(u8g2_font_4x6_tr);
    u8g2.drawStr(99, 37, "MPA MAX");
    u8g2.drawStr(99, 49, "Kn/S");
    u8g2.drawStr(99, 61, "HZ");
    
    u8g2.setFont(u8g2_font_6x12_tr);
    drawRightAligned(97, 37, maxMPA_char);
    drawRightAligned(97, 49, FIR_char);
    drawRightAligned(97, 61, freqRate_char);
    // u8g2.drawStr(69, 37, maxMPA_char);
    // u8g2.drawStr(68, 49, FIR_char);
    // u8g2.drawStr(69, 61, freqRate_char);
    
    
    u8g2.sendBuffer();
  }


// =========================
// Main Loop
// =========================
float P_MPA = -1.0;
int P_BAR = -1;
int P_PSI = -1;
float freqRate = 0.0f;
unsigned long lastSuccessMillis = 0;
bool hasPreviousSuccess = false;
float FIR = -1;
bool connection = false;
int lastHB = 0;
void loop()
{
    int16_t pressureRaw;
    int16_t decimalPosition;

    // Read decimal point register 0x0003
    bool decimalOK = readRegister(
        0x0003,
        decimalPosition
    );

    delay(50);

    // Read pressure register 0x0004
    bool pressureOK = readRegister(
        0x0004,
        pressureRaw
    );

    if (decimalOK && pressureOK)
    {   
        unsigned long now = millis();

        if (hasPreviousSuccess)
        {
            unsigned long elapsed = now - lastSuccessMillis;

            if (elapsed > 0)
            {
                freqRate = 1000.0f / elapsed;
            }
        }
        lastSuccessMillis = now;
        hasPreviousSuccess = true;
        

        Serial.print("Raw pressure: ");
        Serial.println(pressureRaw);

        Serial.print("Decimal position: ");
        Serial.println(decimalPosition);

        // Calculate actual pressure
        float pressure = pressureRaw;

        if (decimalPosition >= 0 && decimalPosition <= 4)
        {
            for (int i = 0; i < decimalPosition; i++)
                pressure /= 10.0;
        }

        Serial.print("Pressure: ");
        Serial.println(pressure, decimalPosition);
        Serial.println("------------------------------");
        connection = true;
        P_MPA = pressure;
        P_BAR = int(pressure * 10);
        P_PSI = int(pressure * 145.0377);
        FIR = updateFIR(P_MPA);
        maxMPA = updateMaxMPA(P_MPA);
    }
    else
    {
        Serial.println("Failed to read transmitter");
        Serial.println("------------------------------");
        Serial.println(P_MPA);
        connection = false;
    }
    if (millis()-lastHB>400){
        HB = !HB;
        lastHB = millis();
    }
    drawScreen(connection, P_MPA, P_BAR, P_PSI, freqRate, FIR, maxMPA);
    delay(25);
}