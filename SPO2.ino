#include <EEPROM.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;
#define SCREEN_WIDTH 128 // OLED width,  in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data


int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
int spo2Limit; // sets limit for spo2 level to beep
int addressSpo2Limit = 0;

// create an OLED display object connected to I2C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

#define BUZZER 16
#define POTENTIOMETER 35
#define BTN_UP 32
#define BTN_DOWN 25
#define BTN_START 33
#define debounceTimeout 50
int startButtonPreviousState = HIGH;
long int lastDebounceTime;

bool isBeep = true;
bool isStart = false;
bool initialReading = false;

void setup()
{
  
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  pinMode(BUZZER, OUTPUT);
  pinMode(BTN_UP,INPUT_PULLUP);
  pinMode(BTN_DOWN,INPUT_PULLUP);
  pinMode(BTN_START,INPUT_PULLUP);

  // Get the SPO2Limit Value
  spo2Limit = EEPROM.read(addressSpo2Limit);
  if(spo2Limit == 0){
    // No value found
    // set a default spo2limit
    spo2Limit = 90;
  }

  // initialize OLED display with I2C address 0x3C
  if (!oled.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("failed to start SSD1306 OLED"));
    while (1);
  }

  oled.display();
  delay(2000); // wait two seconds for initializing
  oled.clearDisplay();
  
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }
  
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void readPulse(){
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps
  //read the first 100 samples, and determine the signal range
  if(!initialReading){
    for (byte i = 0 ; i < bufferLength ; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data
      
      oledPrint(0,0,"INITIAL READING...");
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample
  
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.println(irBuffer[i], DEC);
    }
  }


  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
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

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);

    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
    // Display HR and SPO2 in OLED
    String message = "HR=" + String(heartRate) + "\n" + "SPO2=" +String(spo2) + "%";
    oledPrint(0,0,message);    

    if(spo2 <= spo2Limit){
      if(isBeep){
        noTone(BUZZER);
        tone(BUZZER,1900);
      }else{
        noTone(BUZZER);
      }
      isBeep = !isBeep;     
    }
      initialReading = true;
}

void oledPrint(int x, int y, String message)
{  
  oled.clearDisplay();    
  oled.setTextSize(2);         // set text size
  oled.setTextColor(WHITE);    // set text color
  oled.setCursor(x,y);
  oled.println(message);
  oled.display();
  delay(1);
}

void loop()
{
  // Read the button
  int startButtonPressed = digitalRead(BTN_START);
  // Get the current time
  long int currentTime = millis();
  // check if button is not press
  if(startButtonPressed==HIGH){
    lastDebounceTime = currentTime;
    startButtonPreviousState = HIGH;    
  }

  if((currentTime - lastDebounceTime) > debounceTimeout){
    // Button is pressed
    if(!isStart){
      initialReading = false;
      isStart = true;      
    }else{
      isStart = false;
      noTone(BUZZER);
      delay(1000);
    }
  }

  if(isStart){
    readPulse();
  }
}
