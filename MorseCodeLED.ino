//  Jacob Landerer
//  Revised 8.19.22
//  This program blinks an LED in morse code depicting a phrase given by the user in the serial monitor.
//  Note to self: Baud rates are important!!


String UserPhrase = "";
int unit = 90;
void setup() {
  Serial.begin(9600);
  pinMode(4, OUTPUT);
}
void Dit() {
  digitalWrite(4, HIGH);
  delay(unit); // 1 unit
  digitalWrite(4, LOW);
  delay(unit); // 1 unit
}
void Dah() {
  digitalWrite(4, HIGH);
  delay(unit * 3); // 3 units
  digitalWrite(4, LOW);
  delay(unit); // 1 unit
}

void loop() {
if (Serial.available()) { // If, after the user hits the enter key, there is text in the Serial text box 
    UserPhrase = Serial.readString();
    Serial.print("You said "); Serial.print(UserPhrase);
    UserPhrase.toUpperCase(); // Incerases efficiency because morse code does not differentiate case anyway
    
    // This for loop is included in the preceding if statement because that way, the LED only outputs the Morse code for the one String UserInput.
    // If the for loop was independent of the if statement, it would continuously run using the same String, not prompting another UserInput after it finished,
    // One to be brute-forced however.
    
    for (unsigned int i = 0; i < UserPhrase.length(); i++) {
      // Unsigned variables cannot hold negative values. The computer's logic calls for int i to be unsigned.
      
      if (UserPhrase.charAt(i) == 'A') {
        Dit(); Dah();                         // I could make these all switch cases, but it would be the same number of checks
      }
      else if (UserPhrase.charAt(i) == 'B') {
        Dah(); Dit(); Dit(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'C') {
        Dah(); Dit(); Dah(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'D') {
        Dah(); Dit(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'E') {
        Dit();
      }
      else if (UserPhrase.charAt(i) == 'F') {
        Dit(); Dit(); Dah(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'G') {
        Dah(); Dah(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'H') {
        Dit(); Dit(); Dit(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'I') {
        Dit(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'J') {
        Dit(); Dah(); Dah(); Dah();
      }
      else if (UserPhrase.charAt(i) == 'K') {
        Dah(); Dit(); Dah();
      }
      else if (UserPhrase.charAt(i) == 'L') {
        Dit(); Dah(); Dit(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'M') {
        Dah(); Dah();
      }
      else if (UserPhrase.charAt(i) == 'N') {
        Dah(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'O') {
        Dah(); Dah(); Dah();
      }
      else if (UserPhrase.charAt(i) == 'P') {
        Dit(); Dah(); Dah(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'Q') {
        Dah(); Dah(); Dit(); Dah();
      }
      else if (UserPhrase.charAt(i) == 'R') {
        Dit(); Dah(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'S') {
        Dit(); Dit(); Dit();
      }
      else if (UserPhrase.charAt(i) == 'T') {
        Dah();
      }
      else if (UserPhrase.charAt(i) == 'U') {
        Dit(); Dit(); Dah();
      }
      else if (UserPhrase.charAt(i) == 'V') {
        Dit(); Dit(); Dit(); Dah();
      }
      else if (UserPhrase.charAt(i) == 'W') {
        Dit(); Dah(); Dah();
      }
      else if (UserPhrase.charAt(i) == 'X') {
        Dah(); Dit(); Dit(); Dah();
      }
      else if (UserPhrase.charAt(i) == 'Y') {
        Dah(); Dit(); Dah(); Dah();
      }
      else if (UserPhrase.charAt(i) == 'Z') {
        Dah(); Dah(); Dit(); Dit();
      }
      else if (isSpace(UserPhrase.charAt(i))) {
        delay(unit * 5);                               // uses 7 units (A space in Morse code) because the 5 units here are added to the
      }                                                // delay at the end of this if block that always runs.
      else if (UserPhrase.charAt(i) == '1') {
        Dit(); Dah(); Dah(); Dah(); Dah();
      }
      else if (UserPhrase.charAt(i) == '2') {
        Dit(); Dit(); Dah(); Dah(); Dah();
      }
      else if (UserPhrase.charAt(i) == '3') {
        Dit(); Dit(); Dit(); Dah(); Dah();
      }
      else if (UserPhrase.charAt(i) == '4') {
        Dit(); Dit(); Dit(); Dit(); Dah();
      }
      else if (UserPhrase.charAt(i) == '5') {
        Dit(); Dit(); Dit(); Dit(); Dit();
      }
      else if (UserPhrase.charAt(i) == '6') {
        Dah(); Dit(); Dit(); Dit(); Dit();
      }
      else if (UserPhrase.charAt(i) == '7') {
        Dah(); Dah(); Dit(); Dit(); Dit();
      }
      else if (UserPhrase.charAt(i) == '8') {
        Dah(); Dah(); Dah(); Dit(); Dit();
      }
      else if (UserPhrase.charAt(i) == '9') {
        Dah(); Dah(); Dah(); Dah(); Dit();
      }
      else if (UserPhrase.charAt(i) == '0') {
        Dah(); Dah(); Dah(); Dah(); Dah();
      }
      delay(unit * 2);                            // 2 unit delay between letters of a word, already counting the 1 unit delay at the end of a dit or dah that was just displayed
                                                  // making  a total of 3 units of delay, which is what there is supposed to be between letters of a word.
    }
  }
}
