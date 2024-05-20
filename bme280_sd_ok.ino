#include <TimeLib.h>

#include <BufferedPrint.h>
#include <FreeStack.h>
#include <MinimumSerial.h>
#include <RingBuf.h>
#include <SdFat.h>
#include <SdFatConfig.h>
#include <sdios.h>





 #include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
//#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define SEALEVELPRESSURE_HPA (1013.25)

float alt, alt1, alt2, p, t, h;
float Po = 0;
float Z = 0.0065;
float R = 287.0530;
float g = 9.80665;
float To = 0;
//static const int RXPin = 26, TXPin = 27;
//static const uint32_t GPSBaud = 9600;
const int chipSelect = 4;
//TinyGPSPlus gps;
//SoftwareSerial ss(RXPin, TXPin);

Adafruit_BME280 bme;
//SdFat sdCard;
//SdFile meuArquivo;

void setup() {
  Serial.begin(9600);
// Inicializa o modulo SD
  //if(!sdCard.begin(chipSelect,SPI_HALF_SPEED))sdCard.initErrorHalt();
  
  if (!bme.begin(0x76)) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }
  Po = bme.readPressure() / 100.0F;
  To = bme.readTemperature();
}


void loop() {

  //while (ss.available() > 0)
  //    if (gps.encode(ss.read())){
  //      //displayInfo();
  //      Serial.println("ola");
  //    }
  //  if (millis() > 5000 && gps.charsProcessed() < 10)
  //  {
  //    Serial.println(F("No GPS detected: check wiring."));
  //    while(true);
  //  }

  t = bme.readTemperature();
  p = bme.readPressure() / 100.0F;
  h = bme.readHumidity();
  alt = ((To + 273.15) / Z) * (1 - pow(p / Po, (Z * R / g)));
  alt1 = ((To + 273.15) / Z) * ((pow(p / Po, -((Z * R) / g)) - 1));
  alt2 = 44330 *(1-pow((p/Po), 1/5.255));//
  //alt2=bme.readAltitude(SEALEVELPRESSURE_HPA);

Serial.print("Temperature = ");
Serial.print(t);
Serial.println(" *C");
Serial.print(p);
Serial.print("Pressure = ");
Serial.print(bme.readPressure() / 100.0F);
Serial.println("hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
   Serial.println("m");
  
  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  Serial.println("%");

if (alt<0)
alt=0;
//  Serial.println("Temperatura: " + String(t) + ", Pressão:  " + (String)(p) + " hPa");
//  Serial.println("Altitude: " + String(alt) + ", Altitude 1:  " + String(alt1) +
//                 ", Altitude 2: " + (String)(alt2));
 // SD_Wrte();               
Serial.println();


                 //Serial.println(" Retire o SD em 3 segundos!");
                 delay(1000);


                 /* Serial.println();
                   delay(1000);
                   while (ss.available() > 0)
                    if (gps.encode(ss.read())){
                      displayInfo();
                      Serial.println("ola"); */
}
//void SD_Wrte(){
//  meuArquivo.open("POTATOS.txt", O_RDWR | O_CREAT | O_AT_END);
//  meuArquivo.println("Altitude: " + String(alt) + ", Altitude 1:  " + String(alt1) +
//                 ", Altitude 2: " + (String)(alt2));
//  meuArquivo.println("*********************");                
//meuArquivo.println("Temperatura: " + String(t) + ", Pressão:  " + (String)(p) + " hPa");
//    meuArquivo.println(String(minute())+ "  " + String(second()));  
//      meuArquivo.println("****** NEW **** NEW ****");  
//    meuArquivo.println();   
//    meuArquivo.close();
// 
//
//
//    Serial.println("Altitude: " + String(alt) + ", Altitude 1:  " + String(alt1) +
//                 ", Altitude 2: " + (String)(alt2));
//   Serial.println("ESCREVEU NO CARTÃO");
//
//  delay(500);
//}
//


//void displayInfo()
//{
//  //Serial.print(F("Location: "));
//   Serial.print(("Location: "));
//  if (gps.location.isValid())
//  {
//    Serial.print(gps.location.lat(), 6);
//    Serial.print(F(","));
//    Serial.print(gps.location.lng(), 6);
//     Serial.println(F(",---"));
//    Serial.print(gps.speed.kmph());
//     Serial.println(F("  KM/h"));
//     Serial.print(gps.satellites.value()); // Number of satellites in use (u32)
//     Serial.println(F(" Satelites  "));
//      Serial.print(("alt: "));
//      Serial.println(gps.altitude.meters());
//      Serial.println(gps.speed.mps()); // Speed in meters per second (double)
//      Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)
//     delay(1000);
//  }
//  else
//  {
//    Serial.println(F("INVALID"));
//  }
//}
