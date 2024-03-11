#define LEFT_MOTOR                  6
#define LEFT_ENCODER                3
#define RIGHT_MOTOR                 5
#define RIGHT_ENCODER               2

#define RXLED 17
#define TXLED 30 

// dynamics_data will sweep through a range of PWM values (from LOW_PWM to HIGH_PWM), on intervals of PWM_STEP and SAMPLES_PER_PWM values per PWM value
// we will do this twice: once over a larger range ('coarse'), and once over a smaller range with more data points ('fine')

/*---------------------------*/
/*      CODE BLOCK SID1      */
/*---------------------------*/
// Parameters for sweep of whole PWM range
#define SAMPLING_INTERVAL           500 // in ms
#define SAMPLES_PER_PWM             2
#define LOW_PWM                     50 //inclusive
#define HIGH_PWM                    250 // non inclusive
#define PWM_STEP                    10
/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

/*---------------------------*/
/*      CODE BLOCK SID2      */
/*---------------------------*/
// Parameters for second sweep
// #define SAMPLING_INTERVAL           500 // in ms
// #define SAMPLES_PER_PWM             4
// #define LOW_PWM                     TODO
// #define HIGH_PWM                    TODO
// #define PWM_STEP                    5
/*---------------------------*/
/*---------------------------*/
/*---------------------------*/

// based on the variables defining our PWM range sweep, we create the arrays for wheel position, velocity, and pwm data storage
#define NUM_PWM                     (2*(HIGH_PWM-LOW_PWM)/PWM_STEP)
#define SAMPLE_LEN                  (SAMPLES_PER_PWM*NUM_PWM)
uint16_t lpos[SAMPLE_LEN] = {0};
uint16_t rpos[SAMPLE_LEN] = {0};
uint8_t pwm[SAMPLE_LEN] = {0};


int step_num = 0; // tracks the current iteration of the program
int cur_pwm = HIGH_PWM; // current PWM value; we start at the highest PWM, work our way down to the lowest PWM, and then back up
int dir = -1; // the direction by which we shift our PWM values (currently negative, since we are decreasing)

// counts of the encoder ticks for left and right encoder/wheel respectfully
volatile int left_count = 0;
volatile int right_count = 0;

int led_blinker = 0;
boolean read_data = 0;

void setup() {
  Serial.begin(38400);
  write_pwm(0, 0);
  
  // set up the encoder pins as INPUT to read data and motor pins as OUTPUT to drive motor circuits
  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);

  // enable the LED pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  delay(500);

  // delay until car starts
  write_pwm(0, 0);
  digitalWrite(LED_BUILTIN, HIGH);
  digitalWrite(TXLED, LOW);
  digitalWrite(RXLED, LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);
  digitalWrite(TXLED, HIGH);
  delay(1000);
  digitalWrite(RXLED, HIGH);
  delay(1000);

  pwm_init();
  // start the car by first writing the maximum PWM value to get started, then writing in our desired PWM values
  write_pwm(255, 255);
  delay(500);
  write_pwm(cur_pwm, cur_pwm);
  delay(1000);

  // attach an interrupt to our encoder pins to increment the encoder tick count each time the encoder pin reading changes from low to high
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), flag_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), flag_left, CHANGE);
}

void loop() {
  if (step_num < SAMPLE_LEN) {
    // write our current encoder counts to the lpos and rpos arrays, which track our encoder readings over time      
    lpos[step_num] = left_count;
    rpos[step_num] = right_count;

    // write the new PWM values to the motors
    write_pwm(pwm[step_num], pwm[step_num]);
    step_num++;    

    digitalWrite(RXLED, step_num % 2);
    digitalWrite(TXLED, (step_num + 1) % 2);

  } else {
    // if we have finished data collection, wait for serial input; once recevied, print out the collected data
    write_pwm(0, 0);
    if (Serial.available() > 0) {
      if (Serial.read() == 10){
        print_vals();
        read_data = 1;
      }
    }

    if (!read_data) {
      digitalWrite(LED_BUILTIN, !led_blinker);
      digitalWrite(RXLED, led_blinker);
      digitalWrite(TXLED, led_blinker);
      led_blinker = !led_blinker;
    }
    
  }
  
  delay(SAMPLING_INTERVAL);
}

// helper function to create the array of PWM values we will be testing
void pwm_init() {
  for (int i = 0; i < SAMPLE_LEN; i++) {
    pwm[i] = cur_pwm;
    if (i % SAMPLES_PER_PWM) {
      cur_pwm +=dir * PWM_STEP;
      if (cur_pwm <= LOW_PWM || HIGH_PWM <= cur_pwm) {
        dir *= -1;
      }
    }
  }
}

// helper function to write PWM values to the motors
void write_pwm(int pwm_left, int pwm_right) {
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
}

void flag_left() {
  left_count++;
}

void flag_right() {
  right_count++;
}

// helper function to print out the collected data: PWM value and velocity of each wheel
void print_vals() {
  unsigned char lv;
  unsigned char rv;
  Serial.println("pwm, lv, rv");
  for (int i= 2; i < SAMPLE_LEN; i++) {
    lv = (unsigned char) (lpos[i] - lpos[i-1]);
    rv = (unsigned char) (rpos[i] - rpos[i-1]);
    Serial.print( (int) pwm[i]);
    Serial.print(',');
    Serial.print((int) lv);
    Serial.print(',');
    Serial.print((int) rv);
    Serial.print('\n');
  }
}
