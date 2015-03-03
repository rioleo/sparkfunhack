#include <SPI.h>
#include <Adafruit_NeoPixel.h>

const bool DEBUG = false;

const int SLOWEST_WAIT_MS = 15;
const int FASTEST_WAIT_MS = 1;

int CS = 10;

char POWER_CTL = 0x2D;	//Power Control Register
char DATA_FORMAT = 0x31;
char DATAX0 = 0x32;	//X-Axis Data 0
char DATAx = 0x33;	//X-Axis Data 1
char DATAY0 = 0x34;	//Y-Axis Data 0
char DATAy = 0x35;	//Y-Axis Data 1
char DATAZ0 = 0x36;	//Z-Axis Data 0
char DATAz = 0x37;	//Z-Axis Data 1

//This buffer will hold values read from the ADXL345 registers.
char values[10];
//These variables will be used to hold the x,y and z axis accelerometer values.
int x, x2, delta_x;
int y, y2, delta_y;
int z, z2, delta_z;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, 6, NEO_GRB + NEO_KHZ800);

void setup() {

  SPI.begin();
  //Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  //Create a serial connection to display the data on the terminal.
  Serial.begin(9600);

  strip.begin();
  strip.show(); //Init pixels to off

  //Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  //Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);

  //Put the ADXL345 into +/- 4G range by writing the value 0x01 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x01);
  //Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);  //Measurement mode  

}

double magnitude, magnitude_scale, prev_magnitude_scale, new_magnitude_scale;

void loop() {
  for (int t = 0; t < 60; t++) {
    sinWave(63, 146, 210, 255, 146, 0, 1, t);
    
    // Get the acceleration magnitude
    // Read the accelerometer once
    readRegister(DATAX0, 6, values);
    
    //The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
    //The X value is stored in values[0] and values[1].
    x = (((int)values[1]<<8)|(int)values[0]);
    //The Y value is stored in values[2] and values[3].
    y = (((int)values[3]<<8)|(int)values[2]);
    //The Z value is stored in values[4] and values[5].
    z = (((int)values[5]<<8)|(int)values[4]);
  
    x = map(x, -300, 500, 0, 255);
    y = map(y, -300, 500, 0, 255);
    z = map(z, -300, 500, 0, 255);
    x = constrain(x, 0, 255);
    y = constrain(y, 0, 255);
    z = constrain(z, 0, 255);
    
    if (true) {
      //Print the results to the terminal.
      Serial.print(x, DEC);
      Serial.print(',');
      Serial.print(y, DEC);
      Serial.print(',');
      Serial.println(z, DEC);
    }
    
    magnitude = sqrt(0.0 + sq(x) + sq(y) + sq(z));
    magnitude_scale = (magnitude - 150) / 100;
    
    delay(7);
  }
}

//This function will write a value to a register on the ADXL345.
//Parameters:
//  char registerAddress - The register to write a value to
//  char value - The value to be written to the specified register.
void writeRegister(char registerAddress, char value){
  //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

//This function will read a certain number of registers starting from a specified address and store their values in a buffer.
//Parameters:
//  char registerAddress - The register addresse to start the read sequence from.
//  int numBytes - The number of registers that should be read.
//  char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, char * values){
  //Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  //If we're doing a multi-byte read, bit 6 needs to be set as well.
  if(numBytes > 1)address = address | 0x40;

  //Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  //Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  //Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for(int i=0; i<numBytes; i++){
    values[i] = SPI.transfer(0x00);
  }
  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}

////

void sinWave(int c1R, int c1G, int c1B, int c2R, int c2G, int c2B, int frequency, int t) {
  for (int i = 0; i < 60; i++) {
    double rad = (0.0 + i + t) / 60 * frequency * 2 * PI;
    double sinFactor = (sin(rad) + 1) / 2;
    
    if (DEBUG) {
      Serial.print("i: ");
      Serial.println(i);
    
      Serial.print("sinFactor: ");
      Serial.println(sinFactor);
    }
    
    int cAvgR = (int) (c1R * sinFactor + c2R * (1 - sinFactor));
    int cAvgG = (int) (c1G * sinFactor + c2G * (1 - sinFactor));
    int cAvgB = (int) (c1B * sinFactor + c2B * (1 - sinFactor));
    
    if (DEBUG) {
      Serial.print("cAvgR: ");
      Serial.println(cAvgR);
      Serial.print("cAvgG: ");
      Serial.println(cAvgG);
      Serial.print("cAvgB: ");
      Serial.println(cAvgB);
      
      Serial.println();
    }
    
    uint32_t cAvg = strip.Color(cAvgR, cAvgG, cAvgB);
    strip.setPixelColor(i, cAvg);
  }
  
  strip.show();
}
