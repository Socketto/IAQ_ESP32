#include "Wire.h"
#include "ClosedCube_HDC1080.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <Adafruit_BMP280.h>
#include "SparkFunCCS811.h"

#define AVGCo2 5

#include "DHT.h"

#define DHTPIN 4
#define DHTTYPE DHT11

#define BMP280_I2C_ADDRESS  0x76

Adafruit_BMP280 bmp; // I2C
DHT dht(DHTPIN, DHTTYPE);
float tDHT;
float hDHT;

float tempBMP;
float pressBMP;
float altitudeBMP;

ClosedCube_HDC1080 hdc1080;

int debug_MQTT = 0;

float temperatureVariable = 25.0; //in degrees C
float humidityVariable = 65.0; //in % relative

const char* ssid     = "ssid1";
const char* ssid2    = "ssid2";
const char* password = "pdw";

#define CCS811_ADDR 0x5B //Default I2C Address
#define CCS811_ADDR2 0x5A //Alternate I2C Address
String Body;
CCS811 myCCS811(CCS811_ADDR);
CCS811 myCCS8112(CCS811_ADDR2);

const char* mqtt_server = "192.168.1.12";
WiFiClient espClient;
PubSubClient client(espClient);
String tempString;
char buffertempString[100];

String CO2;
int CO2IsValid = 0;
String CO2_VAR;
float CO2real[AVGCo2];
float temppCO2;
float temppCO2_2;
double Co2Result;
int i;
String TVCO2;
String Temperature;
unsigned long volte = 0;
String Humidity;

void printSensorError()
{
  uint8_t error = myCCS811.getErrorRegister();

  if ( error == 0xFF ) //comm error
  {
    Serial.println("Failed to get ERROR_ID register.");
    volte = 2000;
  }
  else
  {
    Serial.print("Error: ");
    if (error & 1 << 5) Serial.print("HeaterSupply");
    if (error & 1 << 4) Serial.print("HeaterFault");
    if (error & 1 << 3) Serial.print("MaxResistance");
    if (error & 1 << 2) Serial.print("MeasModeInvalid");
    if (error & 1 << 1) Serial.print("ReadRegInvalid");
    if (error & 1 << 0) Serial.print("MsgInvalid");
    Serial.println();
  }
}

void ResetLedRGB()
{
  digitalWrite(25, LOW);
  digitalWrite(26, LOW);
  digitalWrite(27, LOW);
}

void Blue()
{
  digitalWrite(27, HIGH);
}

void Red()
{
  digitalWrite(26, HIGH);
}

void Green()
{
  digitalWrite(25, HIGH);
}


void CheckWIFI()
{
  unsigned char resetRequired = 0;
  while (WiFi.status() != WL_CONNECTED) {
    volte++;
    resetRequired = 1;

    if (volte == 1)
    {
      WiFi.begin(ssid2, password);
      Serial.println("Connecting to ");
      Serial.println(ssid2);
      Serial.println(password);
      Serial.println();
    }
    else
    {
      if (volte == 200)
      {
        WiFi.begin(ssid, password);
        Serial.println("Connecting to ");
        Serial.println(ssid);
        Serial.println(password);
        Serial.println();
      }
      if (volte >= 300)
      {
        volte = 0;
      }
    }

    delay(100);
    Red();
    delay(100);
    Serial.print(".");
    ResetLedRGB();
  }
  if (resetRequired == 1)
  {
    volte = 1995;
  }
}

void setup()
{
  pinMode(19, OUTPUT);

  pinMode(0, INPUT); //boot

  pinMode(2, OUTPUT); //RELE

  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  Red();
  Green();
  Blue();
  digitalWrite(19, LOW); // sets the digital pin 13 on
  delay(500);            // waits for a second
  digitalWrite(19, HIGH); // sets the digital pin 13 on
  delay(2000);            // waits for a second

  Serial.begin(115200);
  Serial.println("CCS811 EnvironmentalReadings Example");
  Serial.println("ClosedCube HDC1080 Arduino Test");

  Wire.begin();

  IPAddress ip(192, 168, 1, 9);
  IPAddress gateway(192, 168, 1, 254);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.config(ip, gateway, subnet);

  if (!bmp.begin(BMP280_I2C_ADDRESS)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


  //This begins the CCS811 sensor and prints error status of .begin()
  int returnCode = myCCS811.begin();
  Serial.println("myCSS811_1 begin exited with: " + returnCode);
  Serial.println("\n");
  returnCode = myCCS8112.begin();
  Serial.println("myCSS811_2 begin exited with: " + returnCode);
  if (returnCode == 0)
  {
    volte = 2000;
  }
  Serial.println(returnCode);
  Serial.println();

  // Default settings:
  //  - Heater off
  //  - 14 bit Temperature and Humidity Measurement Resolutions
  hdc1080.begin(0x40);


  Serial.print("Manufacturer ID=0x");
  Serial.println(hdc1080.readManufacturerId(), HEX); // 0x5449 ID of Texas Instruments
  Serial.print("Device ID=0x");
  Serial.println(hdc1080.readDeviceId(), HEX); // 0x1050 ID of the device


  // Connect to Wi-Fi network with SSID and password

  CheckWIFI();

  client.setServer(mqtt_server, 1883);

  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  dht.begin();
  volte = 2000;
}

void reconnect() {
  int o = 0;
  // Loop until we're reconnected
  while (!client.connected())
  {
    CheckWIFI();
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("Co2BoardClient", "mqtt_user", "password"))
    {
      if (client.connected())
      {
        Serial.println("now is connected");
        delay(250);
        Green();
        delay(500);
        ResetLedRGB();
        volte = 1995;
        return;
      }
      else
      {
        Serial.println("ERROR 1");
      }
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      for (o = 0; o < 6; o++)
      {
        delay(250);
        Red();
        Blue();
        delay(250);
        ResetLedRGB();
      }
    }
  }
}

void loop()
{
  int o;
  CheckWIFI();
  Serial.println();
  humidityVariable = (float)hdc1080.readHumidity();
  temperatureVariable = (float)hdc1080.readTemperature();

  Serial.println("New humidity and temperature:");
  Serial.print("  Humidity: ");
  Humidity = humidityVariable;
  Serial.print(humidityVariable, 2);
  Serial.println("% relative");
  Serial.print("  Temperature: ");
  Temperature = temperatureVariable;
  Serial.print(temperatureVariable, 2);
  Serial.println(" degrees C");
  if (digitalRead(0) == 0)
  {
    digitalWrite(2, HIGH);
    debug_MQTT = 1;
  }
  else
  {
    digitalWrite(2, LOW);
  }

  if (temppCO2 > 1500 || digitalRead(0) == 0)
  {
    volte = 2000;
    temppCO2 = 0;
    CO2IsValid = 0;
  }

  volte = volte + 1;
  if (volte > 2000)
  {
    digitalWrite(2, HIGH);
    ResetLedRGB();
    Blue();
    Green();
    digitalWrite(19, LOW); // sets the digital pin 13 on
    delay(1000);            // waits for a second
    digitalWrite(19, HIGH); // sets the digital pin 13 on
    delay(3000);            // waits for a second
    int returnCode = myCCS811.begin();
    Serial.println("CCS811_1 begin exited with: " + returnCode);
    Serial.println("\n");
    returnCode = myCCS8112.begin();
    Serial.println("CCS811_2 begin exited with: " + returnCode);

    Serial.println(returnCode);
    Serial.println();
    hdc1080.begin(0x40);
    if (!bmp.begin(BMP280_I2C_ADDRESS)) {
      Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    }

    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */


    myCCS811.setEnvironmentalData(humidityVariable, temperatureVariable);
    myCCS8112.setEnvironmentalData(humidityVariable, temperatureVariable);

    //Serial.println("Environmental data applied!");
    myCCS811.readAlgorithmResults(); //Dump a reading and wait
    myCCS8112.readAlgorithmResults(); //Dump a reading and wait
    delay(500);
    ResetLedRGB();
    volte = 0;
    if (returnCode == 0)
    {
      volte = 1990;
    }
    CO2IsValid = 0;
  }
  delay(1000);

  digitalWrite(2, LOW);
  //Serial.println("Environmental data applied!");

  if (myCCS811.dataAvailable() && myCCS8112.dataAvailable())
  {
    //Calling readAlgorithmResults() function updates the global tVOC and CO2 variables
    myCCS811.readAlgorithmResults();
    myCCS8112.readAlgorithmResults();
    temppCO2 = myCCS811.getCO2();
    temppCO2_2 = myCCS8112.getCO2();
    Serial.print("\n\n");
    Serial.print(temppCO2);
    Serial.print("  -  ");
    Serial.print(temppCO2_2);
    Serial.print("\n\n");

    if (temppCO2 > 401 && (abs(temppCO2 - temppCO2_2) < 100))
    {
      Serial.print("CO2[now: ");
      for (i = AVGCo2; i > 0 ; i--)
      {
        CO2real[i] = CO2real[i - 1];
      }
      CO2real[0] = (temppCO2 + temppCO2_2) / 2;
      Co2Result = 0;
      for (i = 0; i < AVGCo2 ; i++)
      {
        Co2Result += CO2real[i];
      }
      Co2Result /= AVGCo2;
      CO2 = Co2Result;
      CO2_VAR = abs(Co2Result - CO2real[0]);
      Serial.print(CO2real[0]);
      Serial.print(" - AVG: ");
      Serial.print(CO2);
      Serial.print("] tVOC[");
      TVCO2 = myCCS811.getTVOC();
      Serial.print(TVCO2);
      Serial.print("] millis[");
      Serial.print(millis());
      Serial.print("]");
      Serial.println();
      CO2IsValid = 1;
    }
    else
    {
      CO2IsValid = 0;
    }
  }
  else if (myCCS811.checkForStatusError())
  {
    printSensorError();
    CO2IsValid = 0;
  }
  if (CO2real[AVGCo2 - 1] != 0 && CO2IsValid == 1)
  {
    Blue();
    tempString = "{";
    tempString += "\"CO2\" : \"" + CO2;
    tempString += "\",";
    tempString += "\"CO2_VAR\" : \"" + CO2_VAR;
    tempString += "\",";
    tempString += "\"TVOC\" : \"" + TVCO2;
    tempString += "\",";
    tempString += "\"Temperature\" : \"" + Temperature;
    tempString += "\",";
    tempString += "\"Hum\" : \"" + Humidity;
    tempString += "\"}";
    tempString.toCharArray(buffertempString, 100);
    while (!client.connected())
    {
      Serial.print("On loop client not connected...");
      reconnect();
    }

    client.publish("CO2Board", buffertempString);



    tempString = "{";
    tempString += "\"TemperatureDHT11\" : \"";
    tDHT = dht.readTemperature();
    tempString += tDHT;
    tempString += "\",";
    tempString += "\"HumDHT11\" : \"";
    hDHT = dht.readHumidity();
    tempString += hDHT;
    tempString += "\",";
    tempString += "\"HeatIndexDHT11\" : \"";
    tempString += dht.computeHeatIndex(tDHT, hDHT, false);
    tempString += "\"}";
    tempString.toCharArray(buffertempString, 100);
    if (!isnan(tDHT) && !isnan(hDHT))
      client.publish("CO2Board_DHT11", buffertempString);


    tempBMP = bmp.readTemperature();
    pressBMP = bmp.readPressure();
    altitudeBMP = bmp.readAltitude(1013.25) + 77;
    if (!isnan(tempBMP) && !isnan(pressBMP))
    {
      tempString = "{";
      tempString += "\"Temp\" : \"";
      tempString += tempBMP;
      tempString += "\",";
      tempString += "\"Press\" : \"";
      tempString += pressBMP / 100;
      tempString += "\",";
      tempString += "\"Alt\" : \"";
      tempString += altitudeBMP;
      tempString += "\"}";
      tempString.toCharArray(buffertempString, 100);
      client.publish("CO2Board_BMP280", buffertempString);

      Serial.print("Temperature = ");
      Serial.print(tempBMP);
      Serial.println(" °C");
      // 2: print pressure
      Serial.print("Pressure    = ");
      Serial.print(pressBMP / 100);
      Serial.println(" hPa");
      // 3: print altitude
      Serial.print("Approx Altitude = ");
      Serial.print(altitudeBMP);
      Serial.println(" m");
    }
    client.loop();
    delay(80);
    ResetLedRGB();
    delay(3000);
  }

  if (Co2Result > 401 && CO2real[AVGCo2 - 1] != 0) //valid
  {
    if (Co2Result < 600)
    {
      Green();
    }
    else
    {
      if (Co2Result < 850)
      {
        Red();
        Green();
      }
      else
      {
        Red();
      }
    }
    delay(150);
    ResetLedRGB();
  }
  else
  {
    if (Co2Result <= 400 || CO2real[AVGCo2 - 1] == 0) //not valid
    {
      for (o = 1; o < 4; o++)
      {
        Red();
        delay(100);
        Blue();
        delay(100);
        ResetLedRGB();
      }
    }
  }
}
