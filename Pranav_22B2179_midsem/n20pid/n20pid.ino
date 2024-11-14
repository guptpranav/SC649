// pinout for L298N motoro driver
#define C1 2
#define C2 3
#define IN3 8
#define IN4 12
#define ENB 9

// memory size for integral term
#define MEMSZ 25

// C1C2 directional pattern for encoder top view
//  CLOCK := 00 10 11 01 00
// ACLOCK := 00 01 11 10 00

short aprev = 0;  // previous C1 encoder state
short bprev = 0;  // previous C2 encoder state
short acurr = 0;  // latest C1 encoder state
short bcurr = 0;  // latest C2 encoder state

const float N = 100000;                                                           // period to update RPM (microseconds)
float tickCounter = 0.0;                                                          // counter for encoder ticks
float ticksPerSecond = 0.0;                                                       // encoder ticks per second
const float shaftGearboxRatio = 25.0;                                             // encoder to motor shaft gear ratio, measured by averaging
const float ticksPerEncoderRotation = 25.0;                                       // ticks per encoder rotation, measured by averaging
const float ticksPerMotorRotation = shaftGearboxRatio * ticksPerEncoderRotation;  // ticks per motor rotation

unsigned short pwmTmicros = 10000; // time period for manual pwm in microseconds
unsigned long currentTime = 0;     // current running time of script
unsigned long lastUpdateTime = 0;  // last time the tick count was updated

float RPM = 0.0;        // user-input RPM
float prevRPM = 0.0;    // previous loop RPM
float currRPM = 0.0;    // current loop RPM
float setpRPM = 60.0;   // RPM setpoint

// PID gain constants
const float KP = 40.0;
const float KI = 25.0;
const float KD = 0.00;

int idx = 0;
float e_P[MEMSZ];
float e_I = 0.0;
float e_D = 0.0;

float dt = 0;       // loop runtime in seconds
float loopT = 0.0;  // loop runtime in microseconds

float duty = 0.0;   // commanded duty cycle
float duty_d = 0.0; // commanded duty cycle's rate of change

void setup() {
  // setting L298 pins to OUTPUT mode
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // setting encoder pins to INPUT mode
  pinMode(C1, INPUT);
  pinMode(C2, INPUT);

  // initialising e_P memory to 1000
  for (idx = 0; idx < MEMSZ; idx++) {
    e_P[idx] = 0.0;
  }
  idx = 0;

  lastUpdateTime = micros();
  Serial.begin(38400);
}

void loop() {
  // scanning from RPM input from user through ArduinoIDE serial monitor
  if (Serial.available() != 0) {
    RPM = Serial.parseFloat();
  }
  setpRPM = (RPM == 0) ? setpRPM : RPM;

  loopT = micros() - currentTime;
  dt = loopT / 1000000;
  
  while (!updateRPM()) {
    currentTime = micros();
    updateEncoderTicks();
  }

  e_I -= e_P[idx] * dt;           // subtracting old integral error
  e_P[idx] = setpRPM - currRPM;   // resetting error in history with new value
  e_I += e_P[idx] * dt;           // integrating latest error into integral
  e_D = (currRPM - prevRPM) / dt; // computing error derivative
  duty_d = KP * e_P[idx] + KI * e_I + KD * e_D; // PID equation
  duty += duty_d * dt;  // commanded duty cycle

  duty = abs(duty) < 255 ? duty : 255 * duty / abs(duty); // clipping duty cycle with saturation

  Serial.print(currRPM, 6);
  Serial.print(",");
  Serial.println(setpRPM);

  if (duty >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN4, HIGH);
    digitalWrite(IN3, LOW);
  }

  analogWrite(ENB, abs(duty));

  // // logic for manual PWM signal
  // short t_on  = pwmTmicros * (abs(duty) / 255);
  // short t_off = pwmTmicros * (1 - abs(duty)/255);
  // digitalWrite(ENB, HIGH); delayMicroseconds(t_on);
  // digitalWrite(ENB, LOW); delayMicroseconds(t_off);

  // resetting integral error and index if memory size reached
  idx += 1;
  if (idx == MEMSZ) {
    idx = 0;
    e_I = 0;
  }

  // updating previous cycle RPM
  prevRPM = currRPM;
}

// updates motor tick count directionally
void updateEncoderTicks() {
  // measuring current encoder states
  acurr = digitalRead(C1);
  bcurr = digitalRead(C2);

  // updating directed tick count
  if ((acurr != aprev) || (bcurr != bprev)) {
    if ((acurr == 1 - bprev) && (bcurr == aprev)) {
      tickCounter -= 1;  // clockwise state transition
    } else {
      tickCounter += 1;  // anticlockwise state transition
    }
  }

  // updating previous encoder states
  aprev = acurr;
  bprev = bcurr;
}

// measuring average RPM value over a second of rotation and returns true when done
bool updateRPM() {
  if (currentTime - lastUpdateTime >= N) {
    ticksPerSecond = tickCounter * (1000000 / N);
    tickCounter = 0;
    currRPM = 6 * ticksPerSecond / ticksPerMotorRotation;  // (360/TPMR) * TPS * (1/60);
    lastUpdateTime = currentTime;
    return true;
  } else {
    return false;
  }
}