/*
 * This code is used to record the gestures as templates and save these templates in EEPROM of the Arduino
 * Mega2560. My method is based on the integration of acceleration to get dispacement. Due to that the 
 * gestures are linear motions, when the final dispalcement is similar to template's, we could unlock the board.
 * The copyright of mpu6050 header comes from 2014 by Korneliusz Jarzebski
 */

#include <Wire.h>
#include <EEPROM.h>
#include <MPU6050.h>

MPU6050 mpu;


/* Initialize some basic data*/
const int buttonPin = 2;  //Initalize Pin2 as the input of button
const int ledPin =  7;    // Initialize Pin7 as the output of button. If the button is pressed, Pin7 is high.
/* Initialize the accelerometer's data. 
* I use Acceleration to calculate displacement: Velocity.Now=Velocity.Previous+(Acceleration.Previous+Acceleration.Now)*SamplingInterval/2,
* and Displacement.Now=Displacement.Previous+(Velocity.Previous+Velocity.Now)*SamplingInterval/2*/
double xAcceFormer=0;double yAcceFormer=0;double zAcceFormer=0;
double xAcceNow=0;double yAcceNow=0;double zAcceNow=0;
double xVelFormer=0;double yVelFormer=0;double zVelFormer=0;
double xVelNow=0;double yVelNow=0;double zVelNow=0;
double xLocFormer=0;double yLocFormer=0;double zLocFormer=0;
double xLocNow=0;double yLocNow=0;double zLocNow=0;

/* Due to the fact that EEPROM only can store usigned int, I need to transform Displacement to int*/
int xLocSave=0;int yLocSave=0;int zLocSave=0;

/* index indicades the sample qunatity of a gesture. In this example, I set 30 samples of a gesture.*/
int index=0;
/* gestureNum indicates the number of different gesture template.*/
int gestureNum=0;

int buttonState = 0;         /*variable for reading the pushbutton status*/

void setup() {
  /* initialize the LED pin as an output:*/
  pinMode(ledPin, OUTPUT);
  /* initialize the pushbutton pin as an input:*/
  pinMode(buttonPin, INPUT);
  Serial.begin(115200);
  
  Serial.println("Initialize MPU6050");

// Initialize MPU6050 and the default scale of accelerometer is +-2g
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // If you want, you can set accelerometer offsets
  // mpu.setAccelOffsetX();
  // mpu.setAccelOffsetY();
  // mpu.setAccelOffsetZ();
  
  checkSettings();
}

void checkSettings()
{
  Serial.println();
  
  Serial.print(" * Sleep Mode:            ");
  Serial.println(mpu.getSleepEnabled() ? "Enabled" : "Disabled");
  
  Serial.print(" * Clock Source:          ");
  switch(mpu.getClockSource())
  {
    case MPU6050_CLOCK_KEEP_RESET:     Serial.println("Stops the clock and keeps the timing generator in reset"); break;
    case MPU6050_CLOCK_EXTERNAL_19MHZ: Serial.println("PLL with external 19.2MHz reference"); break;
    case MPU6050_CLOCK_EXTERNAL_32KHZ: Serial.println("PLL with external 32.768kHz reference"); break;
    case MPU6050_CLOCK_PLL_ZGYRO:      Serial.println("PLL with Z axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_YGYRO:      Serial.println("PLL with Y axis gyroscope reference"); break;
    case MPU6050_CLOCK_PLL_XGYRO:      Serial.println("PLL with X axis gyroscope reference"); break;
    case MPU6050_CLOCK_INTERNAL_8MHZ:  Serial.println("Internal 8MHz oscillator"); break;
  }
  
  Serial.print(" * Accelerometer:         ");
  switch(mpu.getRange())
  {
    case MPU6050_RANGE_16G:            Serial.println("+/- 16 g"); break;
    case MPU6050_RANGE_8G:             Serial.println("+/- 8 g"); break;
    case MPU6050_RANGE_4G:             Serial.println("+/- 4 g"); break;
    case MPU6050_RANGE_2G:             Serial.println("+/- 2 g"); break;
  }  

  Serial.print(" * Accelerometer offsets: ");
  Serial.print(mpu.getAccelOffsetX());
  Serial.print(" / ");
  Serial.print(mpu.getAccelOffsetY());
  Serial.print(" / ");
  Serial.println(mpu.getAccelOffsetZ());
  
  Serial.println();
}

void loop() {
  /* read the state of the pushbutton value:*/
  buttonState = digitalRead(buttonPin);
 /*Get the datee from MPU6050*/
  Vector normAccel = mpu.readNormalizeAccel();
  double Xaxis,Yaxis,Zaxis;
  Xaxis=normAccel.XAxis;
  Yaxis=normAccel.YAxis;
  Zaxis=normAccel.ZAxis;

  /* check if the pushbutton is pressed. If it is, the buttonState is HIGH:*/
  if (buttonState == HIGH) {
    /* turn LED on:*/
    digitalWrite(ledPin, HIGH);
   /* Get 30 samples of a gesture from MPU6050 */
    if (index<30){
      xAcceFormer=xAcceNow;yAcceFormer=yAcceNow;zAcceFormer=zAcceNow;
      xVelFormer=xVelNow;yVelFormer=yVelNow;zVelFormer=zVelNow;
      xLocFormer=xLocNow;yLocFormer=yLocNow;zLocFormer=zLocNow;

      /*Because of the influence of gravity, I need to substract gravity from each data and through my test, the gravty 
      * component on X-axis is 0.4 m^2/s, the gravty component on Y-axis is -0.12 m^2/s*, and the gravty component on 
      * Z-axis is 8.88 m^2/2*/
      xAcceNow=Xaxis+0.4;yAcceNow=Yaxis-0.12;zAcceNow=Zaxis-8.88;

      /*Use integral to compute Velocity by accelerometer*/
      xVelNow=xVelFormer+(xAcceNow+xAcceFormer)*0.05/2;
      yVelNow=yVelFormer+(yAcceNow+yAcceFormer)*0.05/2;
      zVelNow=zVelFormer+(zAcceNow+zAcceFormer)*0.05/2;

      /*Use integral to compute displacement by Velocity*/
      xLocNow=xLocFormer+(xVelNow+xVelFormer)*0.05/2;
      yLocNow=yLocFormer+(yVelNow+yVelFormer)*0.05/2;
      zLocNow=zLocFormer+(zVelNow+zVelFormer)*0.05/2;

      
      index=index+1;

      /*When getting 30 samples of a gesture, a template of a gesture is finished*/
      if(index==29){
        gestureNum=gestureNum+1;
        }
    }

  } else {
    /* turn LED off:*/
    digitalWrite(ledPin, LOW);

    /*After getting a gesture, reset the sample counter of a gesture*/
    index=0;

    /*Begin to store every gestore template in EEPROM*/
    if (xLocNow!=0){
      if (gestureNum>0){
        /*Due to the fact that the value in EEPROM is unsigned interger(0-255)
        * while the data from accelerometer has sign and the displacements are 
        * usually lesss than 1 meter, so I multiply 100 and
        *add 128 to each data.Finally, turn these data into int*/
        xLocSave=(int)(100*xLocNow)+128;
        yLocSave=(int)(100*yLocNow)+128;
        zLocSave=(int)(100*zLocNow)+128;

      /*Write template to EEPROM*/
      EEPROM.write(gestureNum*3-3, xLocSave);
      EEPROM.write(gestureNum*3-2, yLocSave);
      EEPROM.write(gestureNum*3-1, zLocSave);
      }

      /*Reset the variables which are used to calculate the displacement*/
      xAcceNow=0; yAcceNow=0; zAcceNow=0;
      xVelNow=0; yVelNow=0; zVelNow=0;
      xLocNow=0; yLocNow=0; zLocNow=0;
      }
  }
  Serial.print(" Xnorm = ");
  Serial.print(normAccel.XAxis);
  Serial.print(" Ynorm = ");
  Serial.print(normAccel.YAxis);
  Serial.print(" Znorm = ");
  Serial.println(normAccel.ZAxis);
  Serial.print('\n');
  Serial.print(xLocNow);
  Serial.print(' ');
  Serial.print(yLocNow);
  Serial.print(' ');
  Serial.println(zLocNow);



  
  delay(50);

}
