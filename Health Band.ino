#define BLYNK_TEMPLATE_ID "TMPL2QD2qPARP"
#define BLYNK_TEMPLATE_NAME "Health band"
#define BLYNK_AUTH_TOKEN "6Vj98uH8VTss5lNU8ah55rETL5JcS5uf"

#include <Wire.h>
#include <Blynk.h>
#include <WiFi.h> 
#include <WiFiClientSecure.h>
#include <BlynkSimpleEsp32.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "ESP32_MailClient.h"
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define LCD_COLUMNS 16
#define LCD_ROWS 2
#define LCD_ADDRESS 0x27 // Replace with your LCD's I2C address

LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
 
#define REPORTING_PERIOD_MS 30000 // frequency of updates sent to blynk app in ms
 
char auth[] = "6Vj98uH8VTss5lNU8ah55rETL5JcS5uf";             // You should get Auth Token in the Blynk App.
char ssid[] = "Phone";                              // Your WiFi credentials.
char pass[] = "00000000";
 
uint32_t tsLastReport = 0;  //stores the time the last update was sent to the blynk app

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value calcualated as per Maxim's algorithm
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 2; //onboard led on esp32 nodemcu
byte readLED = 19; //Blinks with each data read 

long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute; //stores the BPM as per custom algorithm
int beatAvg = 0, sp02Avg = 0; //stores the average BPM and SPO2 
float ledBlinkFreq; //stores the frequency to blink the pulseLED



#define DHTPIN 32   
#define DHTTYPE    DHT11   

DHT_Unified dht(DHTPIN, DHTTYPE);


#define emailSenderAccount    "health.band.top.eltop@gmail.com"
#define emailSenderPassword   "bsak xvde mpfd rkvm"
#define smtpServer            "smtp.gmail.com"
#define smtpServerPort        465
#define emailSubject          "[ALERT] SUDDEN CHANGE"


// Default Recipient Email Address
String inputMessage = "zeinabmohsen41@gmail.com";
String enableEmailChecked = "checked";
String inputMessage2 = "true";
// Default Threshold Temperature Value
String heart_rate_threshold_high = "50.0";
String heart_rate_threshold_low = "90.0";
String oxygen_level_threshold_high = "96.0";
String oxygen_level_threshold_low = "99.0";

// String lastTemperature;

bool emailSent = false;

SMTPData smtpData;


float avgtemp = 0, avghum = 0, counter = 0;
int heart_rate_threshold_counter = 0, oxygen_level_threshold_counter = 0;

//----------------------------------------Host & httpsPort
const char* host = "script.google.com";
const int httpsPort = 443;
//----------------------------------------


WiFiClientSecure client; //--> Create a WiFiClientSecure object.

String GAS_ID = "AKfycbzl-PNoeSAszT4FquLzBwPmpJiWGJNvZKyOlXCZ8c2whHIwdoOQ6DDWbDQV-GkOefTt"; //--> spreadsheet script ID


#define LED_PIN 25

#define BUZZER_PIN 26


void setup()
{ 
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT); // Set the buzzer pin as an output

  lcd.init();  // Initialize the LCD
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  lcd.backlight();
  lcd.setBacklight(255);  // Set backlight to maximum brightness (255)
  lcd.print("Hello, World!");
  Serial.println("LCD initialized.");

  ledcSetup(0, 0, 8); // PWM Channel = 0, Initial PWM Frequency = 0Hz, Resolution = 8 bits
  ledcAttachPin(pulseLED, 0); //attach pulseLED pin to PWM Channel 0
  ledcWrite(0, 255); //set PWM Channel Duty Cycle to 255
  
  Blynk.begin(auth, ssid, pass);
  Serial.begin(115200);
  
  Serial.print("Initializing Pulse Oximeter..");
  
    // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }


  /*The following parameters should be tuned to get the best readings for IR and RED LED. 
   *The perfect values varies depending on your power consumption required, accuracy, ambient light, sensor mounting, etc. 
   *Refer Maxim App Notes to understand how to change these values
   *I got the best readings with these values for my setup. Change after going through the app notes.
   */
  byte ledBrightness = 50; //Options: 0=Off to 255=50mA
  byte sampleAverage = 1; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 69; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384
  
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

  dht.begin();
  
  // WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }

  client.setInsecure();

}
 // Define a variable to track if the IR sensor is touched
bool irSensorTouched = false;

void loop()
{
  float temp = 0;
  float hum = 0;
  sensors_event_t event;
  // Reset the readings and print an error message if the IR sensor is not touched
  long irValue = particleSensor.getIR();

  if (irValue < 50000)
  {
    Serial.println(" No finger?");
    beatAvg = 0;
    sp02Avg = 0;
    tsLastReport = millis();

  }
  else
  {
    bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps

    // Read the first 100 samples, and determine the signal range
    for (byte i = 0 ; i < bufferLength ; i++) 
    {
      while (particleSensor.available() == false) // do we have new data?
        particleSensor.check(); // Check the sensor for new data

      redBuffer[i] = particleSensor.getIR();
      irBuffer[i] = particleSensor.getRed();
      particleSensor.nextSample(); // We're finished with this sample so move to the next sample
    }

    // Calculate heart rate and SpO2 after the first 100 samples (first 4 seconds of samples)
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

    //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
    while (1)
    {
      irValue = particleSensor.getIR();

      if (irValue < 50000)
      {
        Serial.println(" No finger?");
        beatAvg = 0;
        sp02Avg = 0;
        tsLastReport = millis();
        break;

      }
      Blynk.run();
      //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
      for (byte i = 25; i < 100; i++)
      {
        redBuffer[i - 25] = redBuffer[i];
        irBuffer[i - 25] = irBuffer[i];
      }
    
      //take 25 sets of samples before calculating the heart rate.
      for (byte i = 75; i < 100; i++)
      {
        while (particleSensor.available() == false) //do we have new data?
          particleSensor.check(); //Check the sensor for new data
      
        digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read
      
        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); //We're finished with this sample so move to next sample

        long irValue = irBuffer[i];

        //Calculate BPM independent of Maxim Algorithm. 
        if (checkForBeat(irValue) == true)
        {
          //We sensed a beat!
          long delta = millis() - lastBeat;
          lastBeat = millis();
        
          beatsPerMinute = 60 / (delta / 1000.0);
          beatAvg = (beatAvg+beatsPerMinute)/2;

          if(beatAvg != 0)
            ledBlinkFreq = (float)(60.0/beatAvg);
          else
            ledBlinkFreq = 0;
          ledcWriteTone(0, ledBlinkFreq);
        }
        if(millis() - lastBeat > 10000)
        {
          beatsPerMinute = 0;
          beatAvg = (beatAvg+beatsPerMinute)/2;
          
          if(beatAvg != 0)
            ledBlinkFreq = (float)(60.0/beatAvg);
          else
            ledBlinkFreq = 0;
          ledcWriteTone(0, ledBlinkFreq);
        }
      }
  
      //After gathering 25 new samples recalculate HR and SP02
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

      //Calculates average SPO2 to display smooth transitions on Blynk App
      if(validSPO2 == 1 && spo2 < 100 && spo2 > 0)
      {
        sp02Avg = (sp02Avg+spo2)/2;
      }
      else
      {
        spo2 = 0;
        sp02Avg = (sp02Avg+spo2)/2;;
      }


      dht.temperature().getEvent(&event);
      avgtemp += event.temperature;

      dht.humidity().getEvent(&event);
      avghum += event.relative_humidity;
      
      counter++ ;
      //Send Data to Blynk App at regular intervals
      if (millis() - tsLastReport >= REPORTING_PERIOD_MS)
      { 
        Serial.print("BMP: ");
        Serial.println(beatAvg);
        
        Serial.print("SPo2: ");
        Serial.println(sp02Avg);
      // Display heart rate and SpO2 on LCD
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("BPM: " + String(beatAvg));
        lcd.setCursor(0, 1);
        lcd.print("SpO2: " + String(sp02Avg));
        
        Serial.print(F("Temperature: "));
        // dht.temperature().getEvent(&event);
        // temp = event.temperature;
        avgtemp /= counter;
        Serial.println(avgtemp);

        Serial.print(F("Humidity: "));
        // dht.humidity().getEvent(&event);
        // hum = event.relative_humidity;
        avghum /= counter;
        Serial.println(avghum);

        digitalWrite(LED_PIN, LOW);
        if(beatAvg >= 90 || beatAvg <= 50)
        {
          heart_rate_threshold_counter++;
        }
        else
        {
          heart_rate_threshold_counter = 0;
          digitalWrite(LED_PIN,LOW);
        }

        if(sp02Avg >= 99|| sp02Avg <= 96)
        {
          oxygen_level_threshold_counter++;
        }
        else
        {
          oxygen_level_threshold_counter = 0;
          digitalWrite(LED_PIN,LOW);
        }

        if(oxygen_level_threshold_counter >= 2)
        {
          int avghum2 = int(avghum);
          sendData(avgtemp, avghum, sp02Avg, beatAvg);

          String emailMessage = String("Oxygen Level has been outside the average for the past 2 minutes. ") + String("SPo2 = ") + sp02Avg + String("الحقونا, الحقونا بجد ");
          if(sendEmailNotification(emailMessage)) {
            Serial.println(emailMessage);
            emailSent = true;
          }
          else {
            Serial.println("Email failed to send");
          }    
          oxygen_level_threshold_counter = 0;

          digitalWrite(LED_PIN,HIGH);
          digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
          delay(10000); // Buzzer on time (1 second) - you can adjust this as needed
          digitalWrite(BUZZER_PIN, LOW); // Turn off the buzzer
          // Display alert on LCD
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Oxygen Alert!");
        }

        if(heart_rate_threshold_counter >= 2)
        {
          int avghum2 = int(avghum);
          sendData(avgtemp, avghum, sp02Avg, beatAvg);
          String emailMessage = String("Heart rate has been outside the average for the past 2 minutes. \n") + String("bpm = ") + beatAvg + String("الحقونا, الحقونا بجد ");
          if(sendEmailNotification(emailMessage)) {
            Serial.println(emailMessage);
            emailSent = true;
          }
          else {
            Serial.println("Email failed to send");
          }    
          digitalWrite(LED_PIN,HIGH);
          digitalWrite(BUZZER_PIN, HIGH); // Turn on the buzzer
          delay(10000); // Buzzer on time (1 second) - you can adjust this as needed
          digitalWrite(BUZZER_PIN, LOW); // Turn off the buzzer

           // Display alert on LCD
          lcd.clear();
          lcd.setCursor(0, 0);
          lcd.print("Heart Rate Alert!");
          heart_rate_threshold_counter = 0;
        }

        if(sp02Avg >= 99 || beatAvg >= 90)
        {
          int avghum2 = int(avghum);
          sendData(avgtemp, avghum, sp02Avg, beatAvg);
        }

        
        Blynk.virtualWrite(V0, avgtemp);
        Blynk.virtualWrite(V1, avghum);        
        Blynk.virtualWrite(V3, beatAvg);
        Blynk.virtualWrite(V4, sp02Avg);
        
        tsLastReport = millis();
        
        beatAvg = 0;
        sp02Avg = 0;
        avgtemp = 0;
        avghum = 0;
        counter = 0;
      }
    }
  }
}


bool sendEmailNotification(String emailMessage){
  // Set the SMTP Server Email host, port, account and password
  smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword);

  // For library version 1.2.0 and later which STARTTLS protocol was supported,the STARTTLS will be 
  // enabled automatically when port 587 was used, or enable it manually using setSTARTTLS function.
  //smtpData.setSTARTTLS(true);

  // Set the sender name and Email
  smtpData.setSender("Health Band", emailSenderAccount);

  // Set Email priority or importance High, Normal, Low or 1 to 5 (1 is highest)
  smtpData.setPriority("High");

  // Set the subject
  smtpData.setSubject(emailSubject);

  // Set the message with HTML format
  smtpData.setMessage(emailMessage, true);

  // Add recipients
  smtpData.addRecipient(inputMessage);

  smtpData.setSendCallback(sendCallback);

  // Start sending Email, can be set callback function to track the status
  if (!MailClient.sendMail(smtpData)) {
    Serial.println("Error sending Email, " + MailClient.smtpErrorReason());
    return false;
  }
  // Clear all data from Email object to free memory
  smtpData.empty();
  return true;
}

// Callback function to get the Email sending status
void sendCallback(SendStatus msg) {
  // Print the current status
  Serial.println(msg.info());

  // Do something when complete
  if (msg.success()) {
    Serial.println("----------------");
  }
}


// Subroutine for sending data to Google Sheets
void sendData(float tem, int hum, int sp0, int beat) {
  Serial.println("==========");
  Serial.print("connecting to ");
  Serial.println(host);
  
  //----------------------------------------Connect to Google host
  if (!client.connect(host, httpsPort)) {
    Serial.println("connection failed");
    return;
  }
  //----------------------------------------

  //----------------------------------------Processing data and sending data
  String string_temperature =  String(tem);
  // String string_temperature =  String(tem, DEC); 
  String string_humidity =  String(hum, DEC); 

  String string_beat =  String(beat, DEC); 
  String string_spo =  String(sp0, DEC); 
  
  String url = "/macros/s/" + GAS_ID + "/exec?temperature=" + string_temperature + "&humidity=" + string_humidity + "&rate=" + string_beat + "&spo=" + string_spo;
  Serial.print("requesting URL: ");
  Serial.println(url);

  client.print(String("GET ") + url + " HTTP/1.1\r\n" +
         "Host: " + host + "\r\n" +
         "User-Agent: BuildFailureDetectorESP8266\r\n" +
         "Connection: close\r\n\r\n");

  Serial.println("request sent");
  //----------------------------------------

  //----------------------------------------Checking whether the data was sent successfully or not
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    if (line == "\r") {
      Serial.println("headers received");
      break;
    }
  }
  String line = client.readStringUntil('\n');
  if (line.startsWith("{\"state\":\"success\"")) {
    Serial.println("esp8266/Arduino CI successfull!");
  } else {
    Serial.println("esp8266/Arduino CI has failed");
  }
  Serial.print("reply was : ");
  Serial.println(line);
  Serial.println("closing connection");
  Serial.println("==========");
  Serial.println();
  //----------------------------------------
} 
//==============================================================================