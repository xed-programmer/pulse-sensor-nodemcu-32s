#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data


int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

#define BUZZER 16
#define POTENTIOMETER 35
#define BTN_UP 32
#define BTN_DOWN 25
#define BTN_START 33
bool isBeep = true;
bool isStart = false;
int btn_start = 0;

void setup()
{
  /*
  PROBLEMA SA PAG STOP NG READING NG SENSOR
  */
  Serial.begin(115200); // initialize serial communication at 115200 bits per second:
  pinMode(BUZZER, OUTPUT);
  pinMode(BTN_UP,INPUT_PULLUP);
  pinMode(BTN_DOWN,INPUT_PULLUP);
  pinMode(BTN_START,INPUT_PULLUP);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

//  Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
//  while (Serial.available() == 0) ; //wait until user presses a key
//  Serial.read();
  
  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop()
{
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

Serial.println("Back to loop");
    while(!isStart){        
      // waiting for user to click the start button
        btn_start = digitalRead(BTN_START);
        if(btn_start == LOW){
        // Start the readings
        if(isStart == false){
          isStart = true;
        }else if(isStart == true){
          //stop the readings
          isStart = false;
          break;
        }
      }
    }

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {
    btn_start = digitalRead(BTN_START);

//    if(btn_start == LOW){
//      // Stop the readings
//      if(isStart == false){
//        isStart = true;
//      }else if(isStart == true){
//        isStart = false;
//        break;
//      }
//    }else if(btn_start == HIGH){
//      if(isStart == false){
//        isStart = true;
//        break;
//      }
//    }

    int check = 0;
    if (btn_start == LOW) {
      check++;
    }

    if(check % 2 == 0){
      isStart = true;
    } else {
      isStart = false;
      break;
    }
    
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

//      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

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

      if(isBeep){
        tone(BUZZER,1900);
//        delay(500);
      }else{
        noTone(BUZZER);
      }
      isBeep = !isBeep;
    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  }
}
