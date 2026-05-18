// Waveshare Tutorial IV: Motor Without Encoder Control Demo - relevant pin definitions/snippet.
// Source: https://www.waveshare.com/wiki/Tutorial_II:_Motor_Without_Encoder_Control_Demo

// The following defines the ESP32 pin of the TB6612 control
// Motor A
const uint16_t PWMA = 25;
const uint16_t AIN2 = 17;
const uint16_t AIN1 = 21;

// Motor B
const uint16_t BIN1 = 22;
const uint16_t BIN2 = 23;
const uint16_t PWMB = 26;

const uint16_t ANALOG_WRITE_BITS = 8;
int freq = 100000;
int channel_A = 0;
int channel_B = 1;
int resolution = ANALOG_WRITE_BITS;

void initMotors(){
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  ledcSetup(channel_A, freq, resolution);
  ledcAttachPin(PWMA, channel_A);
  ledcSetup(channel_B, freq, resolution);
  ledcAttachPin(PWMB, channel_B);
}
