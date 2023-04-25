/*
  This program plays music using two octaves of the chromatic scale starting with a c at 131 Hz and ending with a C at 524 Hz
*/
int speakerPin = 10;               //the pin that buzzer is connected to
void setup()
{
  pinMode(speakerPin, OUTPUT);    //set the output pin for the speaker
}

void loop() {
  int beatLength = 20; // songs have different tempos, so beatLength must change to fit the song
  //Also, I originally did the math as if beatLength was the length of an entire 4 beats (one measure in common time). However, it should be the length of a quarter note.
  //Quarter notes were originally beatLength / 4, but they should be beatLength.
  play(12, beatLength / 2);          // Beginning of Beethoven's fifth symphony
  play(12, beatLength / 2);          // Eighth note are (beatLength / 2) because one beatLength is a quarter note
  play(12, beatLength / 2);
  play(9, beatLength * 4);  // Whole note rest
  play(25, beatLength *  6);        //Gives listeners a break between melodies

  beatLength = 12;

  // 12 bar blues chord progression
  // with major seventh arpeggios, which are:
  //Every other natural note instead of the major arpeggio (first, third, fifth and eighth)

  c7();
  c7();  // each method plays the four notes in the respective major seventh chord successively.
  c7();  // this (hopefully) gives the illusion of a single chord being played
  c7();
  f7();
  f7();
  c7();
  c7();
  g7();
  f7();
  c7();
  g7();
play(25, beatLength * 6);


  // Blues "Turnaround" chord progression

  f();
  bFlat();
  f();
  f();
  bFlat();
  bFlat();
  f();
  f();
  c();
  bFlat();
  f();
  c();
beatLength = 15;

// This is a walking bass line I learned on guitar in middle school
play (7, beatLength); // g
play (11, beatLength); // b
play (14, beatLength); // D
play (16, beatLength); // E
play (17, beatLength); // F
play (16, beatLength); // E
play (14, beatLength); // D
play (11, beatLength); // b
play (7, beatLength); // g
play (11, beatLength); // b
play (14, beatLength); // D
play (16, beatLength); // E
play (17, beatLength); // F
play (16, beatLength); // E
play (14, beatLength); // D
play (11, beatLength); // b

play (12, beatLength); // C
play (16, beatLength); // E
play (19, beatLength); // G
play (16, beatLength); // E
play (12, beatLength); // C
play (16, beatLength); // E
play (19, beatLength); // G
play (16, beatLength); // E
play (7, beatLength); // g
play (11, beatLength); // b
play (14, beatLength); // D
play (16, beatLength); // E
play (17, beatLength); // F
play (16, beatLength); // E
play (14, beatLength); // D
play (11, beatLength); // b

  while (true) {}
}

void play( int note, int beats) {
  int beatLength = 20;
  double frequencies[] = {131, 138.5, 147, 155.5, 165, 175, 185, 196, 207.6, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 415.3, 440, 466, 494, 524, 0};
  tone(speakerPin, frequencies[note], beatLength * beats);
  delay(beats * beatLength);
  delay(50);
}
void c7() {
  play (0, 5); // C
  play (4, 5); // E
  play (7, 5); // G
  play (11, 5); // B
}
void f7() {
  play (5, 5); // F
  play (9, 5); // A
  play (12, 5); // C
  play (16, 5); // E
}
void g7() {
  play (7, 5); // G
  play (11, 5); // B
  play (14, 5); // D
  play (17, 5); // F
}
void f() {
  play (5, 5); // F
  play (9, 5); // A
  play (12, 5); // C
  play (17, 5);  // F
}
void bFlat() {
  play (10, 5); // Bb
  play (14, 5); // D
  play (17, 5); // F
  play (22, 5); // Bb
}
void c() {
  play (12, 5); // C
  play (16, 5); // E
  play (19, 5); // G
  play (24, 5); // C
}
/* Reference Chart
  Note      Frequency (Hz)  Index
  c        131              0
  c#       138.5            1
  d        147              2
  d#       155.5            3
  e        165              4
  f        175              5
  f#       185              6
  g        196              7
  g#       207.6            8
  a        220              9
  a#       233              10
  b        247              11
  C        262              12
  C#       277              13
  D        294              14
  D#       311              15
  E        330              16
  F        349              17
  F#       370              18
  G        392              19
  G#       415.3            20
  A        440              21
  A#       466              22
  B        494              23
  C        524              24
*/
