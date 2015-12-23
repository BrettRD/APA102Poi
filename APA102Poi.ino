
#include <SPI.h>
#include "nyanCat18.c"

const int nLeds=36;
unsigned int rownum = 0;
const unsigned char *rowBuffer = &nyanCat_Image.pixel_data[nyanCat_Image.width*3*rownum];
unsigned int imageHeight = nyanCat_Image.height;

SPISettings LEDsettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);
SPISettings MPUsettings = SPISettings(4000000, MSBFIRST, SPI_MODE0);

void inline SPIled(){
  SPI.setMOSI(7);
  SPI.setSCK(14);
  SPI.beginTransaction(LEDsettings);
}
void inline SPImpu(){
  SPI.setMOSI(11);
  SPI.setSCK(13);
  SPI.beginTransaction(MPUsettings);
}



void setLeds(unsigned int rownum){

  rowBuffer = &nyanCat_Image.pixel_data[nyanCat_Image.width*3*rownum];

  SPIled();
  //start message;
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  
  for(int i=0; i<(nLeds/2); i++){
    SPI.transfer(0xE1); //brightness
    //SPI.transfer(0xFF); //brightness
    const unsigned char *pixelBuffer = &rowBuffer[3*i];
    SPI.transfer(pixelBuffer[2]);
    SPI.transfer(pixelBuffer[1]);
    SPI.transfer(pixelBuffer[0]);
  }

  for(int i=(nLeds/2)-1; i>=0; i--){
    SPI.transfer(0xE1); //brightness
    //SPI.transfer(0xFF); //brightness
    const unsigned char *pixelBuffer = &rowBuffer[3*i];
    SPI.transfer(pixelBuffer[2]);
    SPI.transfer(pixelBuffer[1]);
    SPI.transfer(pixelBuffer[0]);
  }

  //end message;
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);
  SPI.transfer(0xFF);

  SPI.endTransaction();

}


void setup() {

  SPI.setMOSI(11);
  SPI.setSCK(13);
  SPI.begin();
  
  SPI.setMOSI(7);
  SPI.setSCK(14);
  SPI.begin();
  
}

void loop() {
  setLeds(rownum);
  delay(1);

  rownum += 1;
  if(rownum>=imageHeight) rownum = 0;

}

