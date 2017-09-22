 /***************************************************************************/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define MIN_PULSE_WIDTH       650
#define MAX_PULSE_WIDTH       2350
#define DEFAULT_PULSE_WIDTH   1500
#define FREQUENCY             50
// our servo # counter
uint8_t servonum = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("16 channel Servo test!");
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);  // Analog servos run at ~60 Hz updates
}

void loop() {
  pwm.setPWM(0, 0, pulseWidth(0));
  Serial.println("0");
  delay(500);
  pwm.setPWM(0, 0, pulseWidth(45));
  Serial.println("45");
  delay(500);
  pwm.setPWM(0, 0, pulseWidth(90));
  Serial.println("90");
  delay(500);
  pwm.setPWM(0, 0, pulseWidth(135));
  Serial.println("135");
  delay(500);
  pwm.setPWM(0, 0, pulseWidth(180));
  Serial.println("180");
  delay(500);
  pwm.setPWM(0, 0, pulseWidth(135));
  Serial.println("135");
  delay(500);
  pwm.setPWM(0, 0, pulseWidth(90));
  Serial.println("90");
  delay(500);
  pwm.setPWM(0, 0, pulseWidth(45));
  Serial.println("45");
  delay(500);
}

int pulseWidth(int angle)
{
  int pulse_wide, analog_value;
  pulse_wide   = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  Serial.println(analog_value);
  return analog_value;
}
/********************************************************************************/
