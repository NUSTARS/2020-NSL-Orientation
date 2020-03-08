#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define HISTORY_LEN 16
#define THRESHOLD 30
#define HISTORY_SAMPLE_DELAY 1000/HISTORY_LEN
#define ROTATION_DELAY_SECONDS 300

bool launched = false;
bool waited = false;
float history[HISTORY_LEN] = {0};
uint8_t idx = 0;

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(9600);

  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  digitalWrite(2, LOW);

  bno.setExtCrystalUse(true);
}


void loop(void)
{
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  if (launched) {
    if (!waited) {
      delay(ROTATION_DELAY_SECONDS * 1000);
      waited = true;
    }
    
      /* Display the floating point data */
    //  Serial.print("X: ");
    //  Serial.print(euler.x());
    //  Serial.print(" Y: ");
    //  Serial.print(euler.y());
    //  Serial.print(" Z: ");
    //  Serial.print(euler.z());
    //  Serial.println("\t\t");
    
      /* Display calibration status for each sensor. */

    digitalWrite(1, HIGH);
    delay(5);
    digitalWrite(1, LOW);
    delay(5);

    if (euler.z() < 0 && euler.x() < 1 && euler.x() > -1) {
      digitalWrite(2, LOW);
    } else {
      digitalWrite(0, euler.x() < 0 ? HIGH : LOW);
      digitalWrite(2, HIGH);
    }

  } else {
    float y = euler.y();
    history[idx++ % HISTORY_LEN] = y > 0 ? y : -y;
    Serial.println((idx - 1) % HISTORY_LEN);
    float sum;
    for (int i = 0; i < HISTORY_LEN; i++) {
      sum += history[i] / HISTORY_LEN;
    }
    Serial.println(sum);
    if (sum > THRESHOLD) {
      launched = true;
    }
    delay(HISTORY_SAMPLE_DELAY);
  }
  
}
