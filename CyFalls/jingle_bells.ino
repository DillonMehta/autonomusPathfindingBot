// Play "Jingle Bells" using the PololuBuzzer play() function with varying note lengths and no spaces between notes

#include <PololuBuzzer.h>

PololuBuzzer buzzer;

void setup() {
  // Start playing "Jingle Bells" with correct note durations and no spaces
  buzzer.play("T180L4V15MS"
              // "Jingle bells, jingle bells, jingle all the way"
              "EEE2EEE2EGCDE1"
              // "Oh, what fun it is to ride in a one-horse open sleigh"
              "F2F2F2F2F2EEEEDDE2D2G1");
}

void loop() {
  // Continuously check if the buzzer needs to play the next note
  buzzer.playCheck();
}