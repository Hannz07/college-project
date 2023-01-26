#include <TinyGPS++.h>                                  
#include <SoftwareSerial.h>                             
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#define BLYNK_PRINT Serial
#include <Adafruit_ssd1306syp.h>
#include <SPI.h>
#include <DHT.h>
#define DHTPIN D2
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

Adafruit_ssd1306syp display(12,13);                       // OLED display (SDA to Pin 4), (SCL to Pin 5)

static const int RXPin = 2, TXPin = 0;                // Ublox 6m GPS module to pins 12 and 13
static const uint32_t GPSBaud = 9600;                   // Ublox GPS default Baud Rate is 9600

//const double Home_LAT = --.-----;                      // Your Home Latitude
//const double Home_LNG = --.-----;                      // Your Home Longitude
unsigned int move_index = 1;                            // fixed location for now
float spd;                                              //Variable  to store the speed
float sats;                                             //Variable to store no. of satellites response
String bearing;                                         //Variable to store orientation or direction of GPS

TinyGPSPlus gps;                                        // Create an Instance of the TinyGPS++ object called gps
SoftwareSerial ss(RXPin, TXPin);                        // The serial connection to the GPS device
WidgetMap myMap(V0);                                    // V0 for vitrual pin of Map Widget
WidgetLCD lcd(V3);
BlynkTimer timer;

char auth[] = "r5CVU279SSK3-f__Kwejjuc00EJxzd_X";                   //Your Project authentication key
char ssid[] = "Test";                              // Name of your WiFi network (HotSpot or Router name)
char pass[] = "12345678"; 

void setup()
{  
  Serial.begin(9600);
  display.initialize();                                 // Initialize OLED display  
  display.clear();                                      // Clear OLED display
  display.setTextSize(1);                               // Set OLED text size to small
  display.setTextColor(WHITE);                          // Set OLED color to White
  display.setCursor(0,0);                               // Set cursor to 0,0
  display.println("Nodemcu GPS Tracker");  
  display.print("Version: ");
  display.println(TinyGPSPlus::libraryVersion());
  display.println("Connecting to WIFI ...");
  display.update();
  Blynk.begin(auth, ssid, pass);
  lcd.clear();
  dht.begin();
  delay(1500);
  Serial.println("WiFi Connected!");
  delay(1500);

  display.println("WiFi Connected!");
  display.update();                                     // Update display
  delay(1500);                                          // Pause 1.5 seconds  
  ss.begin(GPSBaud);                                    // Set Software Serial Comm Speed to 9600    
 

  timer.setInterval(5000L, checkGPS); // every 5s check if GPS is connected, only really needs to be done once
  
  //display.println("WiFi Connected!");
}

void checkGPS(){
  if (gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
      Blynk.virtualWrite(V4, "GPS ERROR");  // Value Display widget  on V4 if GPS not detected
  }
}

void sendSensor()
{
  int h = dht.readHumidity();
  int t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit

  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  // You can send any value at any time.
  // Please don't send more that 10 values per second.
  Blynk.virtualWrite(V5, t);  //V5 is for Humidity
  Blynk.virtualWrite(V6, h);  //V6 is for Temperature
}

void loop()
{  
  Blynk.run();
  timer.run();

  Serial.print("LAT: ");
  Serial.println(gps.location.lat(),5);
  

  Serial.print("LON: ");
  Serial.println(gps.location.lng(),5);
  Serial.println("\n");
   
  display.clear();
  display.setCursor(0,0); 
  display.print("Latitude  : ");
  display.println(gps.location.lat(), 2);
  display.print("Longitude : ");
  display.println(gps.location.lng(), 2);
  display.print("Satellites: ");
  display.println(gps.satellites.value());
  display.print("Elevation : ");
  display.print(gps.altitude.feet());
  display.println("ft"); 
  display.print("Time UTC  : ");
  display.print(gps.time.hour());                       // GPS time UTC 
  display.print(":");
  display.print(gps.time.minute());                     // Minutes
  display.print(":");
  display.println(gps.time.second());                   // Seconds
  display.print("Heading   : ");
  display.println(gps.course.deg());
  display.print("Speed     : ");
  display.print(gps.speed.mps());
  display.println("m/s");
  display.print("---> INDIHOME :) <---");
  display.update();

  while(ss.available() > 0)
  {
    //skecth displays information every time a new sentence is correctly encoded
    //similar to Smartdelay function
    if (gps.encode(ss.read()))
      displayInfo(); 
    
  }
  
  //unsigned long Distance_To_Home = (unsigned long)TinyGPSPlus::distanceBetween(gps.location.lat(),gps.location.lng(),Home_LAT, Home_LNG);
  //display.print("KM to Home: ");                        // Have TinyGPS Calculate distance to home and display it
  //display.print(Distance_To_Home);
  //display.update();                                     // Update display
  //delay(200); 
  
  smartDelay(500);                                      // Run Procedure smartDelay

  if (millis() > 5000 && gps.charsProcessed() < 10)
    display.println(F("No GPS data received: check wiring"));
}

static void smartDelay(unsigned long ms)                // This custom version of delay() ensures that the gps object is being "fed".
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void displayInfo()
{ 
  if (gps.location.isValid() ) 
  {    
    float latitude = (gps.location.lat());     //Storing the Lat. and Lon. 
    float longitude = (gps.location.lng()); 

    int h = dht.readHumidity();
    int t = dht.readTemperature(); // or dht.readTemperature(true) for Fahrenheit
    
    Serial.print("LAT:  ");
    Serial.println(latitude, 3);  // float to x decimal places
    Serial.print("LONG: ");
    Serial.println(longitude, 3);
    Blynk.virtualWrite(V1, String(latitude, 3));   
    Blynk.virtualWrite(V2, String(longitude, 3));  
    myMap.location(move_index, latitude, longitude, "GPS_Location");
    spd = gps.speed.kmph();               //get speed
       Blynk.virtualWrite(V3, spd);
       Blynk.virtualWrite(V4, spd);
       lcd.print(0,0, "Speed:");
       lcd.print(7,0,spd);
       Blynk.virtualWrite(V3, t);
       Blynk.virtualWrite(V5, t);
       lcd.print(0,1, "Temp:");
       lcd.print(5,1,t);  
       //Blynk.virtualWrite(V3, h);
       Blynk.virtualWrite(V6, h);
       //lcd.print(0,1, "Hum:");
       //lcd.print(4,1,h);                  
  }
  
 Serial.println();
}
