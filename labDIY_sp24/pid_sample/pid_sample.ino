/*
 * pid_sample.ino
 * Sample code for a PID Controller
 *
 * EECS16B Spring 2024
 * Venkata Alapati
 */

#define LEFT_MOTOR                  6
#define LEFT_ENCODER                3
#define RIGHT_MOTOR                 5
#define RIGHT_ENCODER               2

#define RXLED 17
#define TXLED 30

#define RUN_TIME                    (20*1000)
#define SAMPLING_INTERVAL           175
#define SAMPLE_LEN                  (RUN_TIME/SAMPLING_INTERVAL)

#define JOLT_STEPS                  2

#define NUM_CONTROLLERS             5
#define BANG_BANG                   0
#define P_CONTROL                   1
#define PI_CONTROL                  2
#define PD_CONTROL                  2
#define PID_CONTROL                 4

int step_num = 0;
volatile int left_count = 0;
volatile int right_count = 0;

volatile int left_position = 0;
volatile int right_position = 0;

/*---------------------------*/
/*      CODE BLOCK CON1      */
/*     Fill in values as     */
/*          needed.          */
/*---------------------------*/

// TODO: Update your values here
float v_star = 60;
float Kp = 2.5;
float Ki = 10;
float Kd = 0.0;
float v_left = 0;
float v_right = 0;
float a_left = 0;
float a_right = 0;
float err_Sum_Left = 0;
float err_Sum_Right = 0;
bool jolted = false;
int control_scheme = PID_CONTROL;

// PWM inputs to jolt the car straight
int left_jolt = 100;
int right_jolt = 100;

void loop() {
  if (step_num < JOLT_STEPS && !jolted) {
    write_pwm(left_jolt, right_jolt);
    step_num++;

    digitalWrite(RXLED, (step_num / 5) % 2);
    digitalWrite(TXLED, ((step_num / 5) + 1) % 2);
  } else  {
    jolted = true;

    if (control_scheme == BANG_BANG) { // Bang_Bang Control
      int left_cur_pwm = BangBang_Controller(v_left);
      int right_cur_pwm = BangBang_Controller(v_right);
    }
    else if (control_scheme == P_CONTROL) { // P_Control
      int left_cur_pwm = P_Controller(v_left);
      int right_cur_pwm = P_Controller(v_right);
    }
    else if (control_scheme == PI_CONTROL) { // PI_Control
      int left_cur_pwm = PI_Controller(v_left, err_Sum_Left);
      int right_cur_pwm = PI_Controller(v_right, err_Sum_Right);
    }
    else if (control_scheme == PD_CONTROL) { // PD_Control
      int left_cur_pwm = PD_Controller(v_left, a_left);
      int right_cur_pwm = PD_Controller(v_right, a_right);
    }
    else { // PID_Control
      int left_cur_pwm = PID_Controller(v_left, err_Sum_Left, a_left);
      int right_cur_pwm = PID_Controller(v_right, err_Sum_Right, a_right);
    }
    
    write_pwm(left_cur_pwm, right_cur_pwm);

    step_num++;
    digitalWrite(RXLED, (step_num / 2000) % 2);
    digitalWrite(TXLED, ((int)(step_num / 2000) + 1) % 2);
          
  }

}

/*---------------------------*/
/*    Controller Function    */
/*       Implementation      */
/*---------------------------*/
int BangBang_Controller(float vel){
  if (vel < v_star){
    return 255;
  }
  return 0;
}

int P_Controller(float vel){
  return min(255,max(0,int(Kp*(v_star-vel))));
}

int PI_Controller(float vel, float err_Sum){
  return min(255,max(0,int(Kp*(v_star-vel)) + int(Ki*err_Sum)));;
}

int PD_Controller(float vel, float accel){
  return min(255,max(0,int(Kp*(v_star-vel)) int(Kd * accel)));;
}

int PID_Controller(float vel, float err_Sum, float accel){
  return min(255,max(0,int(Kp*(v_star-vel)) + int(Ki*err_Sum) - int(Kd * accel)));
}

/*---------------------------*/
/*     Setup function        */
/*---------------------------*/

void setup() {
  Serial.begin(38400);

  pinMode(LEFT_MOTOR, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_MOTOR, OUTPUT);
  pinMode(RIGHT_ENCODER, INPUT);
  
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  delay(500);

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
  
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER), flag_right, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER), flag_left, CHANGE);
  
  // Sets timer interrupt every x ms allowing us to sample our change in encoder ticks to get velocity
  cli();
   
  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1  = 0; //initialize counter value to 0
  OCR1A = 16000000/(1000/(float)SAMPLING_INTERVAL*1024) - 1; // = (16*10^6) / (1*1024) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  TIMSK1 |= (1 << OCIE1A);

  sei();

}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm_left, int pwm_right) {
  Serial.print(v_left);
  Serial.print("\t");
  Serial.print(v_right);
  Serial.println();
  analogWrite(LEFT_MOTOR, (int) min(max(0, pwm_left), 255));
  analogWrite(RIGHT_MOTOR, (int) min(max(0, pwm_right), 255));
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

ISR(TIMER1_COMPA_vect){  

  err_Sum_Left = err_Sum_Left * 0.9 + (v_star-v_left)*SAMPLING_INTERVAL/1000;
  err_Sum_Right = err_Sum_Right * 0.9 + (v_star-v_right)*SAMPLING_INTERVAL/1000;

  float v_left_temp = (float)(left_count - left_position)/(float)(SAMPLING_INTERVAL)*1000;
  float v_right_temp = (float)(right_count - right_position)/(float)(SAMPLING_INTERVAL)*1000;

  a_left = (float)(v_left_temp - v_left)/(float)(SAMPLING_INTERVAL)*1000;
  a_right = (float)(v_right_temp - v_right)/(float)(SAMPLING_INTERVAL)*1000;

  left_position = left_count;
  right_position = right_count;
  
  v_left = v_left_temp;
  v_right = v_right_temp;
  
}