/*
 * motor_test.ino
 *
 * Sends a varying (bounded) input PWM
 * signal to pin 2_0 of the Launchpad.
 * The motor will begin slow, speed up,
 * and slow down again repeatedly.
 *
 * EE16B Fall 2016
 * John Maidens, Emily Naviasky & Nathaniel Mailoa
 *
 * EE16B Fall 2017
 * Andrew Blatner
 *
 * EE16B Spring 2019
 * Mia Mirkovic
 *
 */

#define MOTOR                  6

#define RXLED                       17 // The RX LED has a defined Arduino pin
#define TXLED                       30 // The TX LED has a defined Arduino pin

float pwm = 0;
int dir = 1;

void setup(void) {
  Serial.begin(38400);

  pinMode(MOTOR, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  
  Serial.print("Setup done\n");
  reset_blinker();

  write_pwm(0); // Turn off motor
}

void loop(void) {

  float duty_cycle = pwm/255*100;
  write_pwm(pwm);
  Serial.print("Duty cycle: ");
  Serial.print(duty_cycle,DEC);
  Serial.println("%");
  pwm = pwm + dir*5;
  if (pwm >= 255) {
    dir = -1;
  }
  if (pwm <= 0) {
    dir = 1;
  }
  delay(100);
}

/*---------------------------*/
/*     Helper functions      */
/*---------------------------*/

void write_pwm(int pwm) {
  analogWrite(MOTOR, (int) min(max(0, pwm), 255));
}

void reset_blinker(void) {
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, LOW);
}