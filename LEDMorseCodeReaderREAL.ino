//  Jacob Landerer
//  Revised 3.5.22

int threshold;          
int delaySpeed;          
int photoResistor;       
int onCount;            
int offCount;     
int symbols[] = {0, 1, 2, 3, 4, 5}; //  The number 1 is the only thing in morse code that has 5 symbols but I guess that when I wrote this I expected to have numbers done
boolean on = false;           
int i = 0;         
void setup() {
  Serial.begin(9600);     // The program always overrides this baud rate to 115200 for some reason
  Serial.print("The ambient light is ");
  Serial.println(photoResistor);
  threshold = analogRead(A0) + 20;    
}

void loop() {

  photoResistor = analogRead(A0);   // Every time the program loops, it reads the photoresistor value to know whether or not there is a change from on to off or vise versa.

  if (photoResistor > threshold) {    
    if (on) {       // I guess photoResistor being greater than threshold means the LED is off somehow?      Oh I see now its a photoresistor: Its resistance increases when theres light, and more resistance means it reads as off becasue of the threshold
      on = false;   
                                  //  To determine dot or dash; 1- and 3-count are the only possible things for an 'on' state because 
      if (onCount == 1)           //  Dots are 1 unit
        symbols[i] = 1;          
      else if (onCount == 3)      //  And dashes are 3 
        symbols[i] = 2;       

      if (i < 5)   // Again, max number of symbols in a character even though the number 1 is the only one with 5
        i++;      
      else {                    
        DetermineChar();        
        i = 0;                 
      }
    }
    
    else { // if the LED is off and has been off     
      if (offCount == 10)
        DetermineChar();     // symbols are seperated by I think one space (10 off counts)
      else if (offCount == 30) {
        Serial.print(" ");   // And spaces 30
        offCount = 32;
      }
    if (offCount < 32)
      offCount++;
    delay(delaySpeed);   // This delay corresponds to the delay successive dots or dashes in a single char
  }
}


  else if (photoResistor < threshold) {        // If the photoresistor is on
    if (on)            // If it was on before
      onCount++;                // keep counting how long it is on for

    else { // if it was off before
      on = true;                       // Mark that is is now on
      onCount = 0;                     // Reset the count for symbols
    }
    delay(delaySpeed);           // If the delay occured right before taking a photoresistor reading at the beginning of the program, 
                                 // the delay would stagger the program from being in sync with the blink of the LED 
  }                              // So, the delay should only occur once the LED starts blinking
}

void DetermineChar() {                 // Concatenates the elements in symbols[] into a String, then prints the corresponding letter
  String Letter = (String)symbols[0] + (String)symbols[1] + (String)symbols[2] + (String)symbols[3] + (String)symbols[4] + (String)symbols[5];
                                 //This concatenation is easier than nested if statments, which I originally had. This has less checks and logic, but is slower.
  if (Letter == "120000")        // The ints are all initially set to 0, so if they were not changed, they are 0.
    Serial.print("A");         
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
i = 0;
  //I should add numbers, but only once I complete every other aspect of this program. I did woohoo let's gooooooooooo
}
