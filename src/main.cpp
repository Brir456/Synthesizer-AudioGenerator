
/*  Example of a sound being triggered by MIDI input.

    Demonstrates playing notes with Mozzi in response to MIDI input,
    using  Arduino MIDI library v4.2
    (https://github.com/FortySevenEffects/arduino_midi_library/releases/tag/4.2)

    Circuit:
      MIDI input circuit as per http://arduino.cc/en/Tutorial/Midi
      Note: midi input on rx pin, not tx as in the illustration on the above page.
      Midi has to be disconnected from rx for sketch to upload.
      Audio output on digital pin 9 on a Uno or similar.

    Mozzi documentation/API
    https://sensorium.github.io/Mozzi/doc/html/index.html

    Mozzi help/discussion/announcements:
    https://groups.google.com/forum/#!forum/mozzi-users

    Tim Barrass 2013-14, CC by-nc-sa.
*/

// Code for the keyboard: https://www.youtube.com/watch?v=K-OPme8-BNA
// Code for polyphony: https://github.com/jidagraphy/mozzi-poly-synth

#define MOZZI_CONTROL_RATE 256 // Hz, powers of 2 are most reliable
#define MOZZI_AUDIO_RATE 32768
#define MOZZI_AUDIO_MODE MOZZI_OUTPUT_EXTERNAL_TIMED
#define MOZZI_AUDIO_CHANNELS MOZZI_STEREO

#include <Mozzi.h>
#include <Oscil.h>
#include <tables/saw8192_int8.h>
#include <tables/sin8192_int8.h>
#include <tables/triangle_warm8192_int8.h>
#include <tables/smoothsquare8192_int8.h>
#include <tables/whitenoise8192_int8.h>
#include <tables/saw2048_int8.h>
#include <tables/sin2048_int8.h>
#include <tables/triangle2048_int8.h>
#include <tables/square_no_alias_2048_int8.h>
#include <ADSR.h>
#include <Portamento.h>
#include <mozzi_fixmath.h>
#include <ResonantFilter.h>
#include <SPI.h>

#define matrix1 8 // input pins
#define matrix2 4 // output pins
#define numChars 32
#define numModValues 9

#define WS_pin1 1
#define WS_pin2 2
#define WS_pin3 4

byte numVoices = 0;

char receivedChars[numChars];
bool currentState[matrix1 * matrix2];
bool requestState[matrix1 * matrix2];

byte env2_now = 0;
int LFO1_now = 0;
int LFO2_now = 0;
int outputSignal = 0;

//------------Functions-----------------------------------------
void readKeys(void);
void writeKeys(void);
void checkData(void);
void checkSerial(void);
int distortion(int signal, int amount, bool enabled, int mode);
float detune(float freq, int fine);
void setFreq(void);
void modulator(bool env2, bool lfo1, bool lfo2);

//------------Variables changeable from GUI --------------------

// Global Settings
int OCTAVE = 4;
int SLIDETIME = 50;

// OSC 1
int OSC1_OCT = 0;
int OSC1_SEMI = 0;
int OSC1_LEVEL = 255;
int OSC1_FINE = 0;

// OSC 2
int OSC2_OCT = 0;
int OSC2_SEMI = 0;
int OSC2_LEVEL = 0;
int OSC2_FINE = 0;

// NOISE
int NOISE_LEVEL = 0;

// ENV 1
int ENV1_AL = 255;
int ENV1_DL = 255;
int ENV1_SL = 100;
int ENV1_RL = 0;
int ENV1_A = 20;
int ENV1_D = 500;
int ENV1_S = 5000;
int ENV1_R = 50;

// ENV 2
bool ENV2_STATE = false;
int ENV2_AL = 255;
int ENV2_DL = 255;
int ENV2_SL = 0;
int ENV2_RL = 0;
int ENV2_A = 5;
int ENV2_D = 40;
int ENV2_S = 200;
int ENV2_R = 50;

// LFO 1 + 2
bool LFO1_STATE = false;
float LFO1_FREQ = 0.1f;

bool LFO2_STATE = false;
float LFO2_FREQ = 0.1f;

// Distortion
bool PREDISTSTATE = false;
int PREDISTAMOUNT = 0;
int PREDISTMODE = 0;

bool POSTDISTSTATE = false;
int POSTDISTAMOUNT = 0;
int POSTDISTMODE = 0;

// Filter
int FILTERSTATE = 0;
int FILTERTYPE = 0;
int FILTERCUTOFF = 255;
int FILTERRESONANCE = 5;

int modValues[numModValues] = {
    0, // OSC 1 LEVEL
    0, // OSC 1 FINE
    0, // OSC 2 LEVEL
    0, // OSC 2 FINE
    0, // NOISELEVEL
    0, // PREDISTAMOUNT
    0, // POSTDISTAMOUNT
    0, // FILTERCUTOFF
    0  // FILTERRESONANCE
};

int modulatedValuesOutput[numModValues] = {
    0, // OSC 1 LEVEL
    0, // OSC 1 FINE
    0, // OSC 2 LEVEL
    0, // OSC 2 FINE
    0, // NOISELEVEL
    0, // PREDISTAMOUNT
    0, // POSTDISTAMOUNT
    0, // FILTERCUTOFF
    0  // FILTERRESONANCE

};

int *ptrModValues[numModValues] = {
    &OSC1_LEVEL,
    &OSC1_FINE,
    &OSC2_LEVEL,
    &OSC2_FINE,
    &NOISE_LEVEL,
    &PREDISTAMOUNT,
    &POSTDISTAMOUNT,
    &FILTERCUTOFF,
    &FILTERRESONANCE //
};

int env2VarNdx[numModValues] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
int env2Amount[numModValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int env2ModType[numModValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

int LFO1VarNdx[numModValues] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
int LFO1Amount[numModValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int LFO1ModType[numModValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

int LFO2VarNdx[numModValues] = {-1, -1, -1, -1, -1, -1, -1, -1, -1};
int LFO2Amount[numModValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
int LFO2ModType[numModValues] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

//--------------------------------------------------------------

// OSC 1 + 2
Oscil<SIN8192_NUM_CELLS, MOZZI_AUDIO_RATE> osc1;
Oscil<SIN8192_NUM_CELLS, MOZZI_AUDIO_RATE> osc2;
Oscil<WHITENOISE8192_NUM_CELLS, MOZZI_AUDIO_RATE> noise;

// ENV 1 + 2
ADSR<MOZZI_CONTROL_RATE, MOZZI_AUDIO_RATE> env1;
ADSR<MOZZI_CONTROL_RATE, MOZZI_CONTROL_RATE> env2;

// LFO 1 + 2
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> LFO1;
Oscil<SIN2048_NUM_CELLS, MOZZI_CONTROL_RATE> LFO2;

// Portamento for OSC 1 + 2
Portamento<MOZZI_CONTROL_RATE> slide1;
Portamento<MOZZI_CONTROL_RATE> slide2;

MultiResonantFilter<uint8_t> filter; // Multifilter applied to a 8 bits signal.
                                     // MultiResonantFilter<uint16_t> can also be used for signals with higher number of bits
                                     // in this last case, both the cutoff frequency and the resonance are uint16_t,
                                     // ranging from 0, to 65535.

enum types
{
  lowpass,
  bandpass,
  highpass,
  notch
};

void audioOutput(const AudioOutput f) // f is a structure containing both channels

{

  /* Note:
   *  the digital writes here can be optimised using portWrite if more speed is needed
   */
  uint16_t rightSignal = f.r();
  uint16_t leftSignal = f.l();

  digitalWrite(WS_pin1, LOW); // select Right channel
  digitalWrite(WS_pin2, LOW); // select Right channel
  digitalWrite(WS_pin3, LOW); // select Right channel

  SPI.transfer16(rightSignal); // Note: This DAC works on 0-centered samples, no need to add MOZZI_AUDIO_BIAS

  digitalWrite(WS_pin1, HIGH); // select Left channel
  digitalWrite(WS_pin2, HIGH); // select Left channel
  digitalWrite(WS_pin3, HIGH); // select Right channel

  SPI.transfer16(leftSignal);
}

void setup()
{
  pinMode(WS_pin1, OUTPUT);
  pinMode(WS_pin2, OUTPUT);
  pinMode(WS_pin3, OUTPUT);
  // digitalWrite(WS_pin1, HIGH);
  // digitalWrite(WS_pin2, HIGH);
  // digitalWrite(WS_pin3, HIGH);

  // Initialising the SPI connection on default port
  SPI.begin();
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0)); // MSB first, according to the DAC spec

  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, 15, 16);
  //-----matrix1 number of pins-------
  pinMode(6, INPUT_PULLUP);  // 1
  pinMode(7, INPUT_PULLUP);  // 2
  pinMode(8, INPUT_PULLUP);  // 3
  pinMode(39, INPUT_PULLUP); // 4
  pinMode(40, INPUT_PULLUP); // 5
  pinMode(41, INPUT_PULLUP); // 6
  pinMode(42, INPUT_PULLUP); // 7
  pinMode(5, INPUT_PULLUP);  // 8

  //-----matrix2 number of pins-------
  pinMode(18, INPUT_PULLUP); // 1
  pinMode(13, INPUT_PULLUP); // 2
  pinMode(14, INPUT_PULLUP); // 3
  pinMode(17, INPUT_PULLUP); // 4

  slide1.setTime(SLIDETIME);
  slide2.setTime(SLIDETIME);
  env1.setLevels(ENV1_AL, ENV1_DL, ENV1_SL, ENV1_RL);
  env1.setTimes(ENV1_A, ENV1_D, ENV1_S, ENV1_R);
  env2.setLevels(ENV2_AL, ENV2_DL, ENV2_SL, ENV2_RL);
  env2.setTimes(ENV2_A, ENV2_D, ENV2_S, ENV2_R);
  osc1.setTable(SAW8192_DATA);
  osc2.setTable(SAW8192_DATA);
  noise.setTable(WHITENOISE8192_DATA);
  noise.setFreq((float)MOZZI_AUDIO_RATE / WHITENOISE8192_SAMPLERATE);
  LFO1.setTable(SIN2048_DATA);
  LFO1.setFreq(LFO1_FREQ);
  LFO2.setTable(SIN2048_DATA);
  LFO2.setFreq(LFO2_FREQ);

  startMozzi(MOZZI_CONTROL_RATE);
  Serial.println("Setup done");
}

void updateControl()
{
  checkSerial();
  readKeys();
  writeKeys();

  env1.update();
  env2.update();
  env2_now = env2.next();
  LFO1_now = LFO1.next();
  LFO2_now = LFO2.next();
  modulator(ENV2_STATE, LFO1_STATE, LFO2_STATE);
  setFreq();
  filter.setCutoffFreqAndResonance(modulatedValuesOutput[7], modulatedValuesOutput[8]);
}

AudioOutput updateAudio()
{
  int asig;
  int env1next = env1.next();
  outputSignal = (env1next * ((osc1.next() * modulatedValuesOutput[0] + osc2.next() * modulatedValuesOutput[2]) >> 8) * 3) >> 3;
  outputSignal = distortion(outputSignal, PREDISTAMOUNT, PREDISTSTATE, PREDISTMODE);

  filter.next(outputSignal);
  if (FILTERSTATE)
  {
    switch (FILTERTYPE) // recover the output from the current selected filter type.
    {
    case lowpass:
      outputSignal = filter.low(); // lowpassed sample
      break;
    case highpass:
      outputSignal = filter.high(); // highpassed sample
      break;
    case bandpass:
      outputSignal = filter.band(); // bandpassed sample
      break;
    case notch:
      outputSignal = filter.notch(); // notched sample
      break;
    }
  }
  outputSignal = distortion(outputSignal, POSTDISTAMOUNT, POSTDISTSTATE, POSTDISTMODE);
  if (NOISE_LEVEL != 0)
  {
    outputSignal += (env1next * noise.next() * modulatedValuesOutput[4] >> 8) >> 2;
  }
  asig = outputSignal;
  return StereoOutput::from16Bit(asig, asig);
}

void loop()
{
  audioHook(); // required here
}

/*
///////////////////////////////////////////////////////////////////////////////////
------------------------------Functions--------------------------------------------
\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
*/

//---------------------Effects------------------------------------------

int distortion(int signal, int amount, bool enabled, int mode)
{
  if (enabled)
  {
    amount = 1 + amount / 51;
    if (mode == 0)
    {
      int output = signal * amount;
      if (output > 24500) // almost 3/4 of 2^15 (amplitude goes to max 16bit/2), left a little bit of headroom
      {
        return 24500;
      }
      if (output < -24500)
      {
        return -24500;
      }
      else
      {
        return output;
      }
    }
    if (mode == 1)
    {
      int output = signal * amount;
      if (output > 32768)
      {
        return 32768 - (output - 32768);
      }
      if (output < -32768)
      {
        return -32768 - (output + 32768);
      }
      else
      {
        return output;
      }
    }
  }
  return signal;
}

//---------------------Matrix------------------------------------------

void modulator(bool env2, bool lfo1, bool lfo2)
{
  for (byte i = 0; i < numModValues; i++)
  {
    modValues[i] = 0;
  }
  for (byte i = 0; i < numModValues; i++)
  {
    if (env2)
    {
      int output;

      if (env2VarNdx[i] != -1)
      {

        if (env2ModType[i] == 0)
        {
          output = env2_now * env2Amount[i] >> 8;
          modValues[env2VarNdx[i]] += output;
        }
        else
        {
          output = (env2_now - 128) * env2Amount[i] >> 8;
          modValues[env2VarNdx[i]] += output;
        }
      }
    }

    if (lfo1)
    {
      int output;

      if (LFO1VarNdx[i] != -1)
      {

        if (LFO1ModType[i] == 0)
        {
          output = ((LFO1_now + 128) * LFO1Amount[i]) >> 8;
          modValues[LFO1VarNdx[i]] += output;
        }
        else
        {
          output = (LFO1_now * LFO1Amount[i]) >> 8;
          modValues[LFO1VarNdx[i]] += output;
        }
      }
    }

    if (lfo2)
    {
      int output;

      if (LFO2VarNdx[i] != -1)
      {
        if (LFO2ModType[i] == 0)
        {
          output = ((LFO2_now + 128) * LFO2Amount[i]) >> 8;
          modValues[LFO2VarNdx[i]] += output;
        }
        else
        {
          output = (LFO2_now * LFO2Amount[i]) >> 8;
          modValues[LFO2VarNdx[i]] += output;
        }
      }
    }
  }

  for (byte i = 0; i < numModValues; i++)
  {
    int finalOutput = *ptrModValues[i] + modValues[i];
    if (finalOutput > 255)
      finalOutput = 255;
    else if (finalOutput < -255)
    {
      finalOutput = -255;
    }
    modulatedValuesOutput[i] = finalOutput;
  }
}

//---------------------Keyboard Stuff-----------------------------------
float detune(float freq, int fine)
{
  fine = fine;
  if (fine > 0)
  {
    return 0.0595 * freq * fine / 255; // Approximation for one semitone, exact formula not required here
  }
  if (fine < 0)
  {
    return 0.0561 * freq * fine / 255;
  }
  return 0;
}

void setFreq()
{
  float slideFreq1 = Q16n16_to_float(slide1.next());
  float slideFreq2 = Q16n16_to_float(slide2.next());
  osc1.setFreq(slideFreq1 + detune(slideFreq1, modulatedValuesOutput[1]));
  osc2.setFreq(slideFreq2 + detune(slideFreq2, modulatedValuesOutput[3]));
}

void handleNoteOn(byte note)
{
  byte osc1note = (OCTAVE + OSC1_OCT) * 12 + note + OSC1_SEMI;
  byte osc2note = (OCTAVE + OSC2_OCT) * 12 + note + OSC2_SEMI;
  slide1.start(osc1note);
  slide2.start(osc2note);
  env1.noteOn();
  env2.noteOn();
}

void handleNoteOff()
{
  env1.noteOff();
  env2.noteOff();
}

void readKeys()
{
  // check all matrix1 pins for each matrix2 output
  for (int i = 0; i < matrix2; i++)
  {
    // Set all matrix2 pins as INPUT_PULLUP to avoid floating states
    pinMode(18, INPUT_PULLUP);
    pinMode(13, INPUT_PULLUP);
    pinMode(14, INPUT_PULLUP);
    pinMode(17, INPUT_PULLUP);

    // Activate only one row at a time
    if (i == 0)
    {
      pinMode(18, OUTPUT);
      digitalWrite(18, LOW);
    }
    else if (i == 1)
    {
      pinMode(13, OUTPUT);
      digitalWrite(13, LOW);
    }
    else if (i == 2)
    {
      pinMode(14, OUTPUT);
      digitalWrite(14, LOW);
    }
    else if (i == 3)
    {
      pinMode(17, OUTPUT);
      digitalWrite(17, LOW);
    }

    // Read all the column inputs for the current row
    requestState[i * matrix1 + 0] = !digitalRead(6);
    requestState[i * matrix1 + 1] = !digitalRead(7);
    requestState[i * matrix1 + 2] = !digitalRead(8);
    requestState[i * matrix1 + 3] = !digitalRead(39);
    requestState[i * matrix1 + 4] = !digitalRead(40);
    requestState[i * matrix1 + 5] = !digitalRead(41);
    requestState[i * matrix1 + 6] = !digitalRead(42);
    requestState[i * matrix1 + 7] = !digitalRead(5);
  }
}

void writeKeys()
{
  for (int i = 3; i < matrix1 * matrix2; i++) // The first 3 Keys don't exist on the keyboard
  {
    if (requestState[i] == true && currentState[i] == false)
    {
      static byte note;
      currentState[i] = requestState[i];
      note = 27 - i;
      handleNoteOn(note);
      numVoices++;
    }

    if (requestState[i] == false && currentState[i] == true)
    {
      currentState[i] = requestState[i];
      numVoices--;
      if (numVoices == 0)
        handleNoteOff();
    }
  }
}

//-------------Serial Evaluation-----------------------

void checkSerial()
{
  static bool newData;
  static bool recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  if (Serial1.available() > 0 && newData == false)
  {
    rc = Serial1.read();

    if (recvInProgress == true)
    {
      if (rc != endMarker)
      {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars)
        {
          ndx = numChars - 1;
        }
      }
      else
      {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }

    else if (rc == startMarker)
    {
      recvInProgress = true;
    }
  }
  if (newData == true)
  {
    checkData();
  }
  newData = false;
}

void checkData()
{
  String message = String(receivedChars);
  Serial.println(message);
  String valName;
  int val;

  int colonIndex = message.indexOf(':');
  Serial.println(message);

  if (colonIndex != -1)
  {
    valName = message.substring(0, colonIndex);
    val = message.substring(colonIndex + 1).toInt();
  }
  if (valName == "OSC1_TABLE")
  {
    switch (val)
    {
    case 0:
      osc1.setTable(SAW8192_DATA);
      break;
    case 1:
      osc1.setTable(SIN8192_DATA);
      break;
    case 2:
      osc1.setTable(SMOOTHSQUARE8192_DATA);
      break;
    case 3:
      osc1.setTable(TRIANGLE_WARM8192_DATA);
      break;
    case 4:
      osc1.setTable(WHITENOISE8192_DATA);
      break;
    default:
      break;
    }
  }
  if (valName == "OSC2_TABLE")
  {
    switch (val)
    {
    case 0:
      osc2.setTable(SAW8192_DATA);
      break;
    case 1:
      osc2.setTable(SIN8192_DATA);
      break;
    case 2:
      osc2.setTable(SMOOTHSQUARE8192_DATA);
      break;
    case 3:
      osc2.setTable(TRIANGLE_WARM8192_DATA);
      break;
    case 4:
      osc2.setTable(WHITENOISE8192_DATA);
      break;
    default:
      break;
    }
  }
  if (valName == "LFO1_TABLE")
  {
    switch (val)
    {
    case 0:
      LFO1.setTable(SIN2048_DATA);
      break;
    case 1:
      LFO1.setTable(SAW2048_DATA);
      break;
    case 2:
      LFO1.setTable(SQUARE_NO_ALIAS_2048_DATA);
      break;
    case 3:
      LFO1.setTable(TRIANGLE2048_DATA);
      break;
    default:
      break;
    }
  }
  if (valName == "LFO2_TABLE")
  {
    switch (val)
    {
    case 0:
      LFO2.setTable(SIN2048_DATA);
      break;
    case 1:
      LFO2.setTable(SAW2048_DATA);
      break;
    case 2:
      LFO2.setTable(SQUARE_NO_ALIAS_2048_DATA);
      break;
    case 3:
      LFO2.setTable(TRIANGLE2048_DATA);
      break;
    default:
      break;
    }
  }
  if (valName == "SLIDETIME")
  {
    slide1.setTime(val);
    slide2.setTime(val);
  }
  if (valName == "OCTAVE")
  {
    OCTAVE = val;
  }

  if (valName == "OSC1_OCT")
  {
    OSC1_OCT = val;
  }

  if (valName == "OSC1_SEMI")
  {
    OSC1_SEMI = val;
  }

  if (valName == "OSC1_LEVEL")
  {
    OSC1_LEVEL = val;
  }
  if (valName == "OSC1_FINE")
  {
    OSC1_FINE = val;
  }

  if (valName == "OSC2_OCT")
  {
    OSC2_OCT = val;
  }

  if (valName == "OSC1_SEMI")
  {
    OSC1_SEMI = val;
  }

  if (valName == "OSC2_LEVEL")
  {
    OSC2_LEVEL = val;
  }

  if (valName == "OSC2_FINE")
  {
    OSC2_FINE = val;
  }

  if (valName == "NOISE_LEVEL")
  {
    NOISE_LEVEL = val;
  }

  //-------Envelopes----------------------
  if (valName == "ENV1_AL")
  {
    env1.setAttackLevel(val);
  }
  if (valName == "ENV1_DL")
  {
    env1.setDecayLevel(val);
  }
  if (valName == "ENV1_SL")
  {
    env1.setSustainLevel(val);
  }
  if (valName == "ENV1_RL")
  {
    env1.setReleaseLevel(val);
  }
  if (valName == "ENV1_A")
  {
    env1.setAttackTime(val);
  }
  if (valName == "ENV1_D")
  {
    env1.setDecayTime(val);
  }
  if (valName == "ENV1_S")
  {
    env1.setSustainTime(val);
  }
  if (valName == "ENV1_R")
  {
    env1.setReleaseTime(val);
  }

  if (valName == "ENV2_STATE")
  {
    ENV2_STATE = val;
  }
  if (valName == "ENV2_AL")
  {
    env2.setAttackLevel(val);
  }
  if (valName == "ENV2_DL")
  {
    env2.setDecayLevel(val);
  }
  if (valName == "ENV2_SL")
  {
    env2.setSustainLevel(val);
  }
  if (valName == "ENV2_RL")
  {
    env2.setReleaseLevel(val);
  }
  if (valName == "ENV2_A")
  {
    env2.setAttackTime(val);
  }
  if (valName == "ENV2_D")
  {
    env2.setDecayTime(val);
  }
  if (valName == "ENV2_S")
  {
    env2.setSustainTime(val);
  }
  if (valName == "ENV2_R")
  {
    env2.setReleaseTime(val);
  }
  if (valName == "LFO1_STATE")
  {
    LFO1_STATE = val;
  }
  if (valName == "LFO1_FREQ")
  {
    LFO1.setFreq(val / 10.0f);
  }

  if (valName == "LFO2_STATE")
  {
    LFO2_STATE = val;
  }
  if (valName == "LFO2_FREQ")
  {
    LFO2.setFreq(val / 10.0f);
  }

  // Distortion
  if (valName == "PREDISTAMOUNT")
  {
    PREDISTAMOUNT = val;
  }
  if (valName == "PREDISTMODE")
  {
    PREDISTMODE = val;
  }
  if (valName == "PREDISTSTATE")
  {
    PREDISTSTATE = val;
  }

  if (valName == "POSTDISTAMOUNT")
  {
    POSTDISTAMOUNT = val;
  }
  if (valName == "POSTDISTMODE")
  {
    POSTDISTMODE = val;
  }
  if (valName == "POSTDISTSTATE")
  {
    POSTDISTSTATE = val;
  }

  //-------------Filter----------------------
  if (valName == "FILTERSTATE")
  {
    FILTERSTATE = val;
  }
  if (valName == "FILTERTYPE")
  {
    FILTERTYPE = val;
  }
  if (valName == "FILTERCUTOFF")
  {
    FILTERCUTOFF = val;
  }
  if (valName == "FILTERRESONANCE")
  {
    FILTERRESONANCE = val;
  }

  //--------------Modulator------------------
  if (message.startsWith("ENVVARNDX"))
  {
    if (colonIndex != -1)
    {
      String indexStr = message.substring(colonIndex - 1, colonIndex);

      int index = indexStr.toInt();

      if (index >= 0 && index < numModValues)
      {
        env2VarNdx[index] = val;
      }
    }
  }
  if (message.startsWith("ENVAMOUNT_"))
  {
    if (colonIndex != -1)
    {
      String indexStr = message.substring(colonIndex - 1, colonIndex);

      int index = indexStr.toInt();

      if (index >= 0 && index < numModValues)
      {
        env2Amount[index] = val;
      }
    }
  }
  if (message.startsWith("ENVMODTYPE"))
  {
    if (colonIndex != -1)
    {
      String indexStr = message.substring(colonIndex - 1, colonIndex);

      int index = indexStr.toInt();

      if (index >= 0 && index < numModValues)
      {
        env2ModType[index] = val;
      }
    }
  }

  if (message.startsWith("LFO1VARNDX"))
  {
    if (colonIndex != -1)
    {
      String indexStr = message.substring(colonIndex - 1, colonIndex);

      int index = indexStr.toInt();

      if (index >= 0 && index < numModValues)
      {
        LFO1VarNdx[index] = val;
      }
    }
  }
  if (message.startsWith("LFO1AMOUNT_"))
  {
    if (colonIndex != -1)
    {
      String indexStr = message.substring(colonIndex - 1, colonIndex);

      int index = indexStr.toInt();

      if (index >= 0 && index < numModValues)
      {
        LFO1Amount[index] = val;
      }
    }
  }
  if (message.startsWith("LFO1MODTYPE"))
  {
    if (colonIndex != -1)
    {
      String indexStr = message.substring(colonIndex - 1, colonIndex);

      int index = indexStr.toInt();

      if (index >= 0 && index < numModValues)
      {
        LFO1ModType[index] = val;
      }
    }
  }

  if (message.startsWith("LFO2VARNDX"))
  {
    if (colonIndex != -1)
    {
      String indexStr = message.substring(colonIndex - 1, colonIndex);

      int index = indexStr.toInt();

      if (index >= 0 && index < numModValues)
      {
        LFO2VarNdx[index] = val;
      }
    }
  }
  if (message.startsWith("LFO2AMOUNT_"))
  {
    if (colonIndex != -1)
    {
      String indexStr = message.substring(colonIndex - 1, colonIndex);

      int index = indexStr.toInt();

      if (index >= 0 && index < numModValues)
      {
        LFO2Amount[index] = val;
      }
    }
  }
  if (message.startsWith("LFO2MODTYPE"))
  {
    if (colonIndex != -1)
    {
      String indexStr = message.substring(colonIndex - 1, colonIndex);

      int index = indexStr.toInt();

      if (index >= 0 && index < numModValues)
      {
        LFO2ModType[index] = val;
      }
    }
  }
}