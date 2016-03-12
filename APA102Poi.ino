

/*
mpu9250 breakout pins
VCC       3.3v
GND       GND
SCL/SCLK  SCL
SDA/SDI   MOSI
NCS       CS
AD0/SD0   MISO
INT
FSYNC
AUX-CL
AUX-DA
*/

/*
Teensy 3.0 pinout
13        SCL
12        MISO
11        MOSI
10        CS0
*/



#include <SPI.h>
#include "nyanCat18.c"
//#include "Congress32c3.c"

//********SPI variables
//these set the speed and mode settings for different SPI profiles on the Teensy
SPISettings LEDsettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
SPISettings IMUsettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);

//These inline functions set the pinouts of the SPI port to suit the different devices
void inline SPIled(){
  SPI.setMOSI(7);
  SPI.setSCK(14);
  SPI.beginTransaction(LEDsettings);
}
void inline SPIimu(){
  SPI.setMOSI(11);
  SPI.setSCK(13);
  SPI.beginTransaction(IMUsettings);
}
const int slaveSelectPin = 10;  //only used by the IMU.

//********LED variables

//which side faces the body?
const bool RightHanded = true;
//const bool RightHanded = false;
//the apa102 has 5 bits controlling global brightness.
unsigned char brightness = 0x1F;
unsigned char apaHeader = brightness | 0xE0;


//number of LEDs in the entire device
const int nLeds=36;
int rowNum = 0; //the current row of the image being displayed


//********Grab the actual image file properties
//const unsigned char *rowBuffer = &congress32c3_image.pixel_data[nLeds*3*rowNum];
//unsigned int imageHeight = congress32c3_image.height;
const unsigned char *rowBuffer = &nyanCat_Image.pixel_data[nLeds*3*rowNum];
unsigned int imageHeight = nyanCat_Image.height;


//********IMU variables


struct mpu9250config{
  uint8_t address;
  uint8_t value;
};

const int nConfigs = 8;
mpu9250config mpu9250configs[nConfigs] = {
  {0x19, 0x00},  //25:set divider
  {0x1A, 0x07},  //26:config
  {0x1B, 0x18},  //27:gyro config
  {0x1C, 0x18},  //28:accel config
  {0x1D, 0x08},  //29:accel config2
  {0x1E, 0x0B},  //30:low power config
  {0x1F, 0x00},  //31:wake on motion
  {0x23, 0x00},  //35:fifo
};


//fetch pi to as much resolution as the platform can give us
const float pi = 4*atan(1);

//*********Sort out some physical constants of the string/swing arc etc
//full scale select; FS_SEL = 3
const float gyroDegScale = 1.0/16.4; //LSB/ degrees/s
const float gyroRadScale = gyroDegScale * pi / 180.0; //rad/s/LSB

const float stringLen = 0.4;  //1m string
const float imageCirc = 2*pi*stringLen;
const float pixelSize = 0.006;  //6mm pixels
const int nPixCirc = floor(imageCirc/pixelSize);  //number of pixels in the circumfrence
const float radPerPix = 2*pi/(float)nPixCirc;  //radians per pixel


float partPix = 0; //position through the current pixel, this is the target of gyro integration

uint32_t newtime = 0; //some values to handle elapsed time
uint32_t oldtime = 0;
uint32_t timestep = 0;

const uint32_t motionPeriod = 50;  //motion is captured every this many microseconds
uint32_t lastMoCap = 0;



//********LED functions


void setLeds(int rownum){
  //grab the next row from the image
  rowBuffer = &nyanCat_Image.pixel_data[nLeds*3*rownum];
  //rowBuffer = &congress32c3_image.pixel_data[nLeds*3*rowNum];

  //set the SPI pins to suit the LEDs
  SPIled();
  //start message; (the LEDs want a blank space to start)
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  
  //transmit the row to the LED forwards on the right, or backwards on the left
  if(RightHanded){
    for(int i=0; i<nLeds; i++){
      const unsigned char *pixelBuffer = &rowBuffer[3*i];

      SPI.transfer(apaHeader);      //brightness
      SPI.transfer(pixelBuffer[2]); //B
      SPI.transfer(pixelBuffer[1]); //G
      SPI.transfer(pixelBuffer[0]); //R
    }
  }else{
    for(int i=(nLeds-1); i>=0; i--){
      const unsigned char *pixelBuffer = &rowBuffer[3*i];

      SPI.transfer(apaHeader);      //brightness
      SPI.transfer(pixelBuffer[2]); //B
      SPI.transfer(pixelBuffer[1]); //G
      SPI.transfer(pixelBuffer[0]); //R
    }
  }

  //end message; (the LEDs want a solid space to end)
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  //tell the Teensy's library that we're done here
  SPI.endTransaction();

}





//********IMU functions



uint8_t* FetchRegs(uint8_t address, uint8_t* Dout, int count=1){
   SPIimu();
  digitalWrite(slaveSelectPin,LOW);
//  SPI.transfer(address | 0x80);  //older read only implementation
  SPI.transfer(address);
  for(int i=0; i<count; i++){
    Dout[i] = SPI.transfer(Dout[i]);
  }
  digitalWrite(slaveSelectPin,HIGH);
  
  SPI.endTransaction();
  return Dout;
}

uint8_t* ReadRegs(uint8_t address, uint8_t* Dout, int count=1){
  return FetchRegs((address | 0x80), Dout, count);
}

uint8_t* WriteRegs(uint8_t address, uint8_t* Dout, int count=1){
  return FetchRegs((address & 0x7F), Dout, count);
}

char *hex(uint8_t b){
  static char hex[3];
  hex[2] = '\0';
  hex[0] = "0123456789abcedf"[0x0f & (b>>4)];
  hex[1] = "0123456789abcedf"[0x0f & b];
  return hex;
  
}
  
int16_t val(uint8_t *a){
  int16_t r= ((int16_t)a[0]<<8 | a[1]);
  return r;
}


//Print some debug stuff about the IMU
void printMotion(uint8_t *Din){
  Serial.print("\naccel: ");
  Serial.print(val(&Din[0]));
  Serial.print(",\t");
  Serial.print(val(&Din[2]));
  Serial.print(",\t");
  Serial.print(val(&Din[4]));
  
  Serial.print("\ttemp:  ");
  Serial.print(val(&Din[6]));
  Serial.print("\tgyro:  ");
  Serial.print(val(&Din[8]));
  Serial.print(",\t");
  Serial.print(val(&Din[10]));
  Serial.print(",\t");
  Serial.print(val(&Din[12]));
  Serial.println("");
}






void setup() {



  Serial.begin(115200);
  
  // set the slaveSelectPin as an output:
  pinMode (slaveSelectPin, OUTPUT);
  // initialize SPI:
  SPI.setMOSI(11);
  SPI.setSCK(13);
  SPI.begin();
  
  SPI.setMOSI(7);
  SPI.setSCK(14);
  SPI.begin();
  
  delay(3000);  //wait for IMU to boot
  uint8_t Dout[2];
  Dout[0] = 0xAA;
  uint8_t *Din = Dout;

  //can we communicate with IMU?
  Din = ReadRegs(0x75, Dout, 1);
  if(Din[0] == 0x71){
    Serial.println("read from mpu9250 success!");
  }else{
    while(1){
      Serial.println("read from mpu9250 failed at reset");
    }
  }
  
  //write all of our configurations to the IMU
  for(int i = 0; i<nConfigs; i++){
    WriteRegs(mpu9250configs[i].address, &mpu9250configs[i].value, 1);
  }

  //remember how long we took to boot
  oldtime = micros();
  lastMoCap = oldtime;

  
}

void loop() {
  //how long has elepsed since the beginning of the last loop or boot?
  oldtime = newtime;
  newtime = micros();
  timestep = newtime-oldtime;
  
  //don't run the integral too often, come back later if you have to.
  if(newtime - lastMoCap >  motionPeriod){
    lastMoCap += motionPeriod;
    
    //grab some memory to hold information about how we're moving
    uint8_t Dout[18] = {0}; //set it to zeroes
    uint8_t *Din = Dout;  //get a pointer to the first element
    Din = ReadRegs(0x3B, Dout, 18);  //read all of the motion data off the IMU chip into memory
    float radPerSec = gyroRadScale*val(&Din[12]); //gyroZ axis (12th byte) measures the plane
    partPix += radPerSec*((float)timestep/ 1000000.0) / radPerPix;  //integrate the motion to a real angle

    //if something went wrong with the integral, say so over serial.
    if(partPix > 2.0){
      Serial.println("overflow");
    }else if(partPix < -2.0){
      Serial.println("underflow");
    }
    
    
    //if the integral has advanced by a whole pixel, advance the row buffer through the image
    if(partPix > 1.0){
      partPix -= 1;
      rowNum  += 1;
      //Serial.print("++rowNum = ");
      //Serial.println(rowNum);
      if(rowNum>=(int)imageHeight) rowNum = 0;  //loop if the image is done
      setLeds(rowNum);
    }else if(partPix < -1.0){ //hysteresis for change of direction
      partPix += 1;
      rowNum  -= 1;
      //Serial.print("--rowNum = ");
      //Serial.println(rowNum);
      if(rowNum<0) rowNum = imageHeight - 1;
      setLeds(rowNum);
    }
  }
  
  
}

