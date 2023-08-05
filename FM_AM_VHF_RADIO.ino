// Using software gets rid of the need to use a superheterodyne design, but it comes
// at computational cost. However, actual monetary cost of superhet components may make this 
// a fair trade. If I have a hardware component that can supplant software, that would be
// Beneficial for computation time. However If I do not have and cannot afford it, having the
// software is fine

// in setup(), Init LCD display, encoder pins, other necessary pins like for speaker
// in main loop, determine mode, read the rotation of the encoder
// To change channel and determine frq around which to apply bandpass to isolate carrier & audio

// In AM, print 'AM' and current frq in kHz
// read signal from antenna
// apply narrow bandpass to isolate carrier frq & amplify carrier+audio
// LPF to isolate audio signal & send to speaker

// In FM, print""
// read signal from antenna
// Apply bandpass considering +/- 75 kHz modulation
// FFT yields isolated audio, send to speaker

// VHF: print "VHF" & channel number
// transmit & recieve

// Things to consider:
// computational efficiency, can some code be substituted for hardware?
// Finite impulse response, Infinite impulse repsonse, filter parameters
// Noise reduction techniques like noise gating or adaptice noise cancelling
// Use signal processing libraries

// HandleEncoderInput() -- Determines what rotary encoder does based on mode
// SwitchMode() -- changes radio mode (FM,AM,VHF) based on button pushes
// isSkippedChannel(vhfChannel) -- Determines what channel whould be switched to next of rotary encoder is moved in VHF mode bc some are skipped
// isSupportedMode(vhfChannel,duplex,fullDuplex) -- VHF stuff
// AMBandpassFilter(inputSignal)
// FMBandpassFilter(inputSignal)
// handleVHFFrequency()
// handleVHFMode()
// AMLowpassFilter(inputSignal)
// FMAudioDemod(inputSignal)
// VHFTrasmit(audioSignal)
// printLCD()
// HandleVolume Control()

#include <LiquidCrystal.h>
#include <math.h>
#include <Encoder.h>
#include <FFT.h>
const int lcdCol = 16;
const int lcdRow = 2;
LDC lcd(12,11,5,4,3,2);

const int encoderPinA = 6;
const int encoderPinB = 7;
const int FMButtPin = 8; // Pins on the microcontroller to which the hardware is connected
const int AMButtPin = 9;
const int VHFButtPin = 10;

// For AM & FM
float currentFrq;

// For VHF
int currentChannel;
bool boostEnabled = 0;
bool isTrasmitting = 0;

// Volume Control
const int volPin = A0;
int currentVol;

// AM range & bandpass filters to exclude frqs outside the ranges of each radio mode
const float AMFrqMin = 530.0;
const float AMFrqMax = 1710.0;     // kHz
const float AMLowCornerFrq = 529.5;
const float AMHighCornerFrq = 1710.5;
float AMLowPassCutoff; // Depends on chosen frq determined by encoder

// FM ""
const float FMFrqMin = 87500;
const float FMFrqMax = 108000;     // kHz
const float FMLowCornerFrq = 87000;
const float FMHighCornerFrq = 108500; // For Bandpass
float FMNarrowBandLow;
float FMNarrowBandHigh; // Bounds for attenuation and amplification once a carrier frq is chosen

int samplingRate = 10000;
float dt = 1.0 / samplingRate;

int analogData;
int analogInputPin = A0;
int AudioOutPin = 9;
float filteredAudio;
int audioValue;

Encoder myencoder(EncoderPinA,encoderPinB);
enum mode {FM, AM, VHF};
mode currentMode;
LiquidCrystal lcd(12,11,5,4,3,2);

void setup() {
  lcd.begin(lcdCol, lcdRow);
  lcd.print("  My DIY Radio  ")

  myencoder.begin();

  pinMode(FMButtPin, IPUT_PULLUP);
  pinMode(AMButtPin, INPUT_PULLUP);
  pinMode(VHFButtPin, INPUT_PULLUP);
}

void loop() {

// SEEK TRACK

  handleButtonInput();
  handleEncoderInput();

  switch(currentMode){

    case AM:
      ExtractAMAudio(currentFrq);
      break;
    case FM:
      ExtractFMAudio(currentFrq);
      break;
  }

  adjustVol();

  // if(currentMode = VHF && isTrasmitting){
  //   trasmitAudio();
  // }
}

void handleEncoderInput(){

  int encoderVal = myencoder.getValue();

  switch (currentMode){
  case AM // kHz
    // Needs to handle the frq reaching bounds of 530.0 kHz or 1710.0 kHz and looping around...the encoderVal variable accounts for neg values
    if (currentFrq + 10 * encoderVal > 1710.0){
      currentFrq = 530.0 + 10 * encoderVal;
    }
    else if (currentFrq + 10 * encoderVal < 530.0){
      currentFrq = 1710.0 + 10 * encoderVal;
    }
    else 
      currentFrq += 10 * encoderVal; // kHz
  break;

  case FM //MHz
    if (currentFrq + 0.2 * encoderVal > 108.0){
      currentFrq = 87.5 + 0.2 * encoderVal;
    }
    else if (currentFrq + 0.2 * encoderVal < 87.5){
      currentFrq = 108.0 + 0.2 * encoderVal;
    }
    else 
      currentFrq += 0.2 * encoderVal; // MHz
  break;

  //case VHF
  // consider the duplex and full-duplex that occurs given a channel's frq's
  // It's confusing
}
}

void ExtractAMAudio(currentFrq){

  float tau = 1.0 / (2*PI*currentFrq);
  float alpha = dt / (tau + dt);

  while (true){

    analogData = analogRead(A_Pin);

    filteredAudio = alpha * analogData + (1- alpha) * filteredAudio;

    audioValue = filteredAudio;
    analogWrite(AudioOutPin,audioValue);
    delayMicroseconds(1000000 / samplingRate);
  }

}

void ExtractFMAudio(currentFrq){
  float FrqDev = 0.075; // MHz
  
  // Bandpass TF coeffs
  float a0 = 1.0;
  float a1 = 0.0;
  float a2 = -1.0;
  float b1 = -2 * cos(2*PI*currentFrq*dt);
  float b2 = 1.0;

  while (true){

    analogData = analogRead(analogInputPin);
    filteredAudio = (a0 * analogData) + (a1 * filteredAudio) + (a2 * filteredAudio) - (b1 * filteredAudio) - (b2*filteredAudio);
    audioValue = sin(2*PI*currentFrq * dt);
    int audioOut = map(audioValue, -1.0,1.0, 0,255);
    analogWrite(AudioOutPin, audioOut);
    delayMicroseconds(1000000 / samplingRate);
  }
}
