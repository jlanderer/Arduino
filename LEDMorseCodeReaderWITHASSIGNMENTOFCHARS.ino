// Jacob Landerer
// Revised 3.5.22
// This program uses a photoresistor to translate the blinks from an LED in Morse cose into phrases.
// Could be used in tandem with the Morse code display program.

// I think this is the real one, and I made the one with 'REAL' first, but made a new program instead of modifying that 'REAL' one.
// To show that this was really the reall one, and also having a program already say "REAL', I would have written something else in caps

int threshold;
int photoResistor;
int timeOffStart;
int timeOffEnd;
int timeOnStart;
int timeOnEnd;
int unit = 100;
int char1 = 0;
int char2 = 0;
int char3 = 0;
int char4 = 0;
int char5 = 0;
int char6 = 0;
boolean on = false;
int turn = 0;
void setup() {
  Serial.begin(9600);
  photoResistor = analogRead(A0);
  Serial.print("The ambient light is ");
  Serial.println(photoResistor);
  threshold = photoResistor + 20; // Threshold cannot be the initial photoResistor reading because in the loop, the reading fluxuates slightly even with a steady light source.
}
void loop() {
  photoResistor = analogRead(A0);              
  if (photoResistor > threshold && !on) {       
    on = true;                                   //This block of code determines if the LED switched from on to off, and reassigns variables accordingly 
    timeOffEnd = millis();                   
    timeOnStart = millis();                


    Serial.print("and the turn is: ");
    Serial.println(turn);

    if (timeSpace != timeOffEnd) {                         // FIX THE CONDITION!!!!! and then include "amount of time off" block in it
      int timeSpace = timeOffEnd - timeOffStart;   // Calculate the amount of time the LED was off for to distinguish between types of Morse spaces (EXCEPT FOR THE INITIAL OFF PHASE)
      Serial.print("The amount of time off is: ");
      Serial.println(timeSpace);
      if (turn == 2 && timeSpace == unit)                    // This if block determines what type of space was just detected. No need to include turn == 1
        turn++;                                            // because the first char cannot be a space.
                                                             // Turn increments the char that a dot or dash is assigned to
      else if (turn == 2 && timeSpace == 3 * unit)         // If a space between letters is detected
        DetermineChar(char1, char2, char3, char4, char5, char6);

      else if (turn == 2 && timeSpace == 7 * unit) {             // The DetermineChar function should be called whenever a space between words is detected
        Serial.print(" ");                                       // It resets char values to 0
        turn = 0;                                                // Alternatively, I can just print a space and manually reset the char values to 0
        char1 = 0;                                               // only in this case because I know what I need to print
        char2 = 0;
        char3 = 0;
        char4 = 0;
        char5 = 0;
        char6 = 0;
      }
      else if (turn == 3 && timeSpace == unit)                // This loop does not enter the remaining if statements if the turn is set to 0
        turn++;                                               // and instead repeats back to the beginning to take in the next 6 chars

      else if (turn == 3 && timeSpace == 3 * unit)
        DetermineChar(char1, char2, char3, char4, char5, char6);

      else if (turn == 3 && timeSpace == 7 * unit) {
        Serial.print(" ");
        turn = 0;
        char1 = 0;
        char2 = 0;
        char3 = 0;
        char4 = 0;
        char5 = 0;
        char6 = 0;
      }
      else if (turn == 4 && timeSpace == unit)
        turn++;

      else if (turn == 4 && timeSpace == 3 * unit) {
        DetermineChar(char1, char2, char3, char4, char5, char6);
      }
      else if (turn == 4 && timeSpace == 7 * unit) {
        Serial.print(" ");
        turn = 0;
        char1 = 0;
        char2 = 0;
        char3 = 0;
        char4 = 0;
        char5 = 0;
        char6 = 0;
      }
      else if (turn == 5 && timeSpace == unit)
        turn++;

      else if (turn == 5 && timeSpace == 3 * unit)
        DetermineChar(char1, char2, char3, char4, char5, char6);


      else if (turn == 5 && timeSpace == 7 * unit) {
        Serial.print(" ");
        turn = 0;
        char1 = 0;
        char2 = 0;
        char3 = 0;
        char4 = 0;
        char5 = 0;
        char6 = 0;
      }
      else if (turn == 6 && timeSpace == 7 * unit)
        Serial.print(" ");
      turn = 0;
      char1 = 0;
      char2 = 0;
      char3 = 0;
      char4 = 0;
      char5 = 0;
      char6 = 0;
    }
  }

  else if (photoResistor < threshold && on) { // if the photoresistor reading is less than the threshold

    on = false;
    timeOnEnd = millis();
    timeOffStart = millis();
    int timeBlink = timeOnEnd - timeOnStart;
    Serial.print("The amount of time on is: ");
    Serial.println(timeBlink);
    Serial.print("and the turn is: ");
    Serial.println(turn);
    if (turn == 1 && timeBlink == unit)               // This if block determines which chars are dits and which are dahs.
      char1 = 1;
    else if (turn == 1 && timeBlink == 3 * unit)
      char1 = 2;
    else if (turn == 2 && timeBlink == unit)
      char2 = 1;
    else if (turn == 2 && timeBlink == 3 * unit)      // DetermineChar() is not called because it should only be called if the end of a letter is detected.
      char2 = 2;                                      // Therefore, DetermineChar() should only be called whne discerning spaces (the preceding if block).
    else if (turn == 3 && timeBlink == unit)
      char3 = 1;
    else if (turn == 3 && timeBlink == 3 * unit)
      char3 = 2;
    else if (turn == 4 && timeBlink == unit)
      char4 = 1;
    else if (turn == 4 && timeBlink == 3 * unit)
      char4 = 2;
    else if (turn == 5 && timeBlink == unit)
      char5 = 1;
    else if (turn == 5 && timeBlink == 3 * unit)
      char5 = 2;
  }
}
void DetermineChar(int char1, int char2, int char3, int char4, int char5, int char6) {
  String Letter = (String)char1 + (String)char2 + (String)char3 + (String)char4 + (String)char5 + (String)char6;
  // This concatenates all of the chars into one String for comparison
  // instead of nested if statements only concerned with individual chars.

  if (Letter == "120000")        // The ints are all initially set to 0, so if they were not changed, they are 0.
    Serial.print("A");           // Those that are changed are either a 1 (dot) or a 2 (dash).
  else if (Letter == "211100")
    Serial.print("B");
  else if (Letter == "212100")
    Serial.print("C");
  else if (Letter == "211000")
    Serial.print("D");
  else if (Letter == "100000")
    Serial.print("E");
  else if (Letter == "112100")
    Serial.print("F");
  else if (Letter == "221000")
    Serial.print("G");
  else if (Letter == "111100")
    Serial.print("H");
  else if (Letter == "110000")
    Serial.print("I");
  else if (Letter == "122200")
    Serial.print("J");
  else if (Letter == "212000")
    Serial.print("K");
  else if (Letter == "121100")
    Serial.print("L");
  else if (Letter == "220000")
    Serial.print("M");
  else if (Letter == "210000")
    Serial.print("N");
  else if (Letter == "222000")
    Serial.print("O");
  else if (Letter == "122100")
    Serial.print("P");
  else if (Letter == "221200")
    Serial.print("Q");
  else if (Letter == "121000")
    Serial.print("R");
  else if (Letter == "111000")
    Serial.print("S");
  else if (Letter == "200000")
    Serial.print("T");
  else if (Letter == "112000")
    Serial.print("U");
  else if (Letter == "111200")
    Serial.print("V");
  else if (Letter == "122000")
    Serial.print("W");
  else if (Letter == "211200")
    Serial.print("X");
  else if (Letter == "212200")
    Serial.print("Y");
  else if (Letter == "221100")
    Serial.print("Z");
}
