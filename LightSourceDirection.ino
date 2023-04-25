// Using three photoresistors, this program can tell from which side light is coming.


int threshold;
int initialRightPhotoResistor;
int initialLeftPhotoResistor;
void setup() {
  pinMode(9, OUTPUT); // RED
  pinMode(10, OUTPUT); // GREEN
  pinMode(11, OUTPUT); // BLUE
  Serial.begin(9600);
  initialRightPhotoResistor = analogRead(A0);
  initialLeftPhotoResistor = analogRead(A2);
}

void loop() {
  delay(1000);
  int rightPhotoResistor = analogRead(A0);
  int leftPhotoResistor = analogRead(A2);
  int rightSideDifference = rightPhotoResistor - initialRightPhotoResistor;
  int leftSideDifference = leftPhotoResistor - initialLeftPhotoResistor;
  int differenceOfDifference = leftSideDifference - rightSideDifference;

  if (differenceOfDifference > -5 && differenceOfDifference < 5) {
    Serial.println("The light is in the middle");
    Red();
  }
  else if (rightSideDifference > leftSideDifference || -1 * rightSideDifference > leftSideDifference) {
    Serial.println("The light is on the right side");
    Green();
  }
  else if (rightSideDifference < leftSideDifference || -1 * rightSideDifference < leftSideDifference) {
    Serial.println("The light is on the left side");
    Blue();
  }
}

void Red() {
  digitalWrite(9, HIGH);
  digitalWrite(10, LOW);
  digitalWrite(11, LOW);
}
void Green() {
  digitalWrite(9, LOW);
  digitalWrite(10, HIGH);
  digitalWrite(11, LOW);
}
void Blue() {
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  digitalWrite(11, HIGH);
}
