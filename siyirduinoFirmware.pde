////////////////////////////////////////////////////
// Firmware of arduino board for siyirduino game  //
// Ozgur.Cobanoglu@cern.ch, CERN, PH-ESE-ME, 2010 //
/////////////////////////////////////////////ISOTDAQ

// Accelerometer device analog input
#define ACCEL_X_PIN 4
#define ACCEL_Y_PIN 3
#define ACCEL_Z_PIN 2

// Accelerometer sensitivity display
#define ACCEL_GSEL_PIN 7

// RGB LED outputs, active high
#define RED_PIN 5
#define GREEN_PIN 6
#define BLUE_PIN 10

//__________________________________________________________
void setup()
{
    // Configure accelerometer g-rating
  pinMode(ACCEL_GSEL_PIN, OUTPUT);
  digitalWrite(ACCEL_GSEL_PIN, LOW); // 1.5g range when low, 6g when high
  
    // Send data via serial port
  Serial.begin(9600);
}

//__________________________________________________________
static void setRGB(uint8_t r, uint8_t g, uint8_t b)
{
  analogWrite(RED_PIN, r ? 16 : 0);
  analogWrite(GREEN_PIN, g ? 16 : 0);
  analogWrite(BLUE_PIN, b ? 16 : 0);
}

//__________________________________________________________
void loop()
{
  uint8_t x, y, z;

    // Read the 3 accelerometer axes and light the RGB LED color
  x = (uint8_t) (analogRead(ACCEL_X_PIN)>>2);
  y = (uint8_t) (analogRead(ACCEL_Y_PIN)>>2);
  z = (uint8_t) (analogRead(ACCEL_Z_PIN)>>2);
  if ((x>y) && (x>z)) {
    setRGB(1, 0, 0);
  } else if ((y>x) && (y>z)) {
    setRGB(0, 1, 0);
  } else if ((z>y) && (z>x)) {
    setRGB(0, 0, 1);
  } else {
    setRGB(0, 0, 0);
  }  // xxx.87.yyy.45.zzz.56.xyz.\n
  Serial.print("xxx"); Serial.print("."); Serial.print(x, DEC); Serial.print(".");
  Serial.print("yyy"); Serial.print("."); Serial.print(y, DEC); Serial.print(".");
  Serial.print("zzz"); Serial.print("."); Serial.print(z, DEC); Serial.print(".");
  Serial.print("xyz"); Serial.println();
  delay(10);
}
