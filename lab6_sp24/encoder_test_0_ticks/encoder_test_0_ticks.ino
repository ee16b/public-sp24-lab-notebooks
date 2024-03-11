// at the top of the file, we decide which pins correspond to each motor/encoder
#define LEFT_MOTOR                  6
#define LEFT_ENCODER                3
#define RIGHT_MOTOR                 5
#define RIGHT_ENCODER               2

// LED_BUILTIN already defined
#define RXLED                       17 // The RX LED has a defined Arduino pin
#define TXLED                       30 // The TX LED has a defined Arduino pin

#define TIMEOUT 250 // does this even do anything, we can delete this right

#define MAX_TICK_ERROR              4
#define SAMPLING_INTERVAL           250
#define SAMPLES_PER_PHASE           8
#define NUM_PHASES                  4

#define SAMPLE_LEN                  (SAMPLES_PER_PHASE*NUM_PHASES)

// We define our 3 modes/phases of the test:
#define MODE_DRIVE                  0 // Car is running the test and driving the wheels
#define MODE_STOP                   1 // Car has finished completing the test
int loop_mode = -1;

// counts of the encoder ticks for left and right encoder/wheel respectfully
volatile int left_count = 0; //in static RAM
volatile int right_count = 0; //in static RAM

// Our test will have 4 phases: no wheels running, left wheel running, right wheel running, no wheel running
int step_num = 0;
int phase = 0;
int phase_pwms[2][NUM_PHASES] = {
  {0, 240, 0, 240},
  {0, 0, 240, 240},
};

int phase_results[NUM_PHASES][2] = {{0}};

int left_pwm = 0;
int right_pwm = 0;

void setup(void) {
  // set the baud rate as 38400
  Serial.begin(38400);
  while (!Serial) ;
  Serial.println("starting setup");

  // set up the encoder pins as INPUT to read data and motor pins as OUTPUT to drive motor circuits
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  // enable the LED pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // everytime our encoder readings go from low to high, we send an interrupt to our arduino to increment the encoder tick count
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), flag_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), flag_left, CHANGE);

  // ensure motors aren't moving, then begin waiting for button press to begin test
  write_pwm(0, 0);
  step_num = 0;
  left_count = 0;
  right_count = 0;
  start_drive_mode();
  Serial.println("done setup");
}

void loop(void) {
  if (loop_mode == MODE_DRIVE) {
    // blink the LED's while testing
    // RXLED and TXLED turning on correspond to the left motors and right motors moving, respectively
    for (int i = 0; i < 4; i++) {
      if (phase == 1 || phase == 3) digitalWrite(RXLED, step_num % 2);
      if (phase == 2 || phase == 3) digitalWrite(TXLED, step_num % 2);
    }
     
    write_pwm(phase_pwms[0][phase], phase_pwms[1][phase]);

    // counter for how long each phase should be. Once we've reached the desired duration, continue to the next phase (or conclude the test if there are no more phases)
    step_num++;
    if (step_num == SAMPLES_PER_PHASE) {
      show_phase_results();
      phase++;
      if (phase == NUM_PHASES) {
        stop_tests();
      }
      else {
        next_phase();
      }
    }

    delay(SAMPLING_INTERVAL);
  }
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

// Switch to testing mode from listen mode
void start_drive_mode(void) {
  loop_mode = MODE_DRIVE;
  write_pwm(0, 0);
  Serial.println("PICK YOUR CAR UP.");
  for (int i = 0; i < 5; i ++) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
  }
  delay(3000);
}

// Reset counters for next phase
void next_phase(void) {
  Serial.print("Starting phase ");
  Serial.print(phase);
  Serial.print("... ");
  step_num = 0;
  left_count = 0;
  right_count = 0;
}

void show_phase_results(void) {
  write_pwm(0, 0);
  Serial.println(left_count);
  Serial.println(right_count);
  int left_res = left_count;
  int right_res = right_count;
  phase_results[phase][0] = left_res;
  phase_results[phase][1] = right_res;

  if ((0 <= left_res && left_res <= MAX_TICK_ERROR) &&
      (0 <= right_res && right_res <= MAX_TICK_ERROR)) {
    digitalWrite(RXLED, HIGH);
    digitalWrite(TXLED, LOW);
    if (phase == 0) {
      Serial.println("Success.");
    } else {
      Serial.println("Success if encoder wheels are off. Failure if encoder wheels are on.");
    }
  }
  else {
    digitalWrite(RXLED, LOW);
    digitalWrite(TXLED, HIGH);
    if (phase == 0) {
      Serial.println("Failure.");
    } else {
      // check if the respective side of the car moved, exclusively
      // i.e. if we are checking the left wheel, only that moved, and vice versa
      bool left_error = (phase == 1 && right_res >= MAX_TICK_ERROR);
      bool right_error = (phase == 2 && left_res >= MAX_TICK_ERROR);
      Serial.print("Failure if encoder wheels are off. ");
      if (left_error) {
        Serial.println("Detected right wheel moved when left wheel was expected to. Your encoders might be swapped.");
      } else if (right_error){
        Serial.println("Detected left wheel moved when right wheel was expected to. Your encoders might be swapped.");
      } else {
        Serial.println("Success if encoder wheels are on.");
      }
    }
  }
  delay(1000);
}

void stop_tests(void) {
  loop_mode = MODE_STOP;
  write_pwm(0, 0);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(RXLED, HIGH);
  digitalWrite(TXLED, HIGH);
  Serial.println("Tests finished.");
  print_results();
}

void print_results(void) {
  Serial.println("Encoder ticks per phase:");
  for (int i = 0; i < NUM_PHASES; i++) {
    Serial.print(phase_results[i][0]);
    Serial.print("\t");
    Serial.println(phase_results[i][1]);
  }
}

//good
void write_pwm(int pwm_left, int pwm_right) {
  left_pwm = (int) min(max(0, pwm_left), 255);
  right_pwm = (int) min(max(0, pwm_right), 255);
  analogWrite(LEFT_MOTOR, left_pwm);
  analogWrite(RIGHT_MOTOR, right_pwm);
}

void flag_left() {
  if(digitalRead(LEFT_ENCODER)) {
    left_count++;
  }
}

void flag_right() {
  if(digitalRead(RIGHT_ENCODER)) {
    right_count++;
  }
}
