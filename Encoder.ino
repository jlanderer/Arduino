/* Encoder Library - Basic Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */

#include <Encoder.h>

// Change these two numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder myEnc(5, 6);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  //Serial.println("Basic Encoder Test:");
}

void loop() {
  byte buf[4];buf[0] = 0;buf[1] = 0;buf[2] = 0;buf[3] = 0;
  Serial.write(buf, sizeof(buf));
  Serial.print(myEnc.read());Serial.println("");
  }
