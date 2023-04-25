/**
  I don't exactly know what this does other than print a sinusoid & a photoresistor reading simultaneously.
  
  Use digital PWM (pulse with modulation) pin
  &
  analogWrite(pin, level) level is 0 to 255.
  255 corresponds to 5 volts
  use 2 volts as midline for sinusoidal wave (PWM unit of 102)
  level_Double is 102 + 102(sin(2pi- (i/10))  i/10 because 10 points in the cycle from 0 to 2pi

  Use a delay of 5 because we need 10 data points over 20 hertz
  20 hertz is 1/20 of a second
  10 points over 1/20 of a second is one point every 5 milliseconds
**/
static const double pi = PI;
int i = 0;
double levelDouble;
void setup() {
 Serial.begin(9600);
  pinMode(10, OUTPUT);
}
void loop() {
  if (i < 10) {
    delay(5);
    double levelDouble = 102 + 102 * sin((2.0 * pi - i) / 10);
    int levelInt = floor(levelDouble);
    analogWrite(10, levelInt);
    i++;
  }
else if (i == 10)
    i = 0;
int photoResistor = analogRead(A0);   
Serial.println(photoResistor);
}
