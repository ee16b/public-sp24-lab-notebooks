/*
 * collect-data-envelope.ino
 * Data collection sketch for SIXT33N Voice Classification Lab
 *
 * EE16B Spring 2016
 * Emily Naviasky & Nathaniel Mailoa
 *
 * EE 16B Fall 2017
 * Andrew Blatner
 */

#define MIC_INPUT                   A2
#define SIZE                        5504
#define ADC_TIMER_MS                0.35
#define AVG_SHIFT                   5
#define AVG_SIZE                    (int) pow(2, AVG_SHIFT)
#define SIZE_AFTER_FILTER           (int) SIZE / AVG_SIZE

#define RXLED  17
#define TXLED  30 

//data array and index pointer
int16_t out[SIZE_AFTER_FILTER] = {0};

int16_t re0[AVG_SIZE] = {0};
int16_t re1[AVG_SIZE] = {0};
volatile int re_pointer = 0;
int write_arr = 0;

void envelope(int16_t* data, int16_t* data_out, int index){
  int32_t avg = 0;
  for (int i = 0; i < AVG_SIZE; i++) {
      avg += data[i];
  }
  
  avg = avg >> AVG_SHIFT;
  data_out[index] = abs(data[0] - avg);  
  
  for (int i = 1; i < AVG_SIZE; i++) {
      data_out[index] += abs(data[i] - avg);
  }
}

void setup(void) {
  //microphone ADC
  pinMode(MIC_INPUT, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RXLED, OUTPUT);
  pinMode(TXLED, OUTPUT);
  delay(1000);

  re_pointer = 0;

  cli();
   
  TCCR1A = 0;// set entire TCCR1A register to 0
  TCCR1B = 0;// same for TCCR1B
  TCNT1  = 0;//initialize counter value to 0
  OCR1A = 15.624 * ADC_TIMER_MS;// = (16*10^6) / (1*1024) - 1 (must be <65536)
  TCCR1B |= (1 << WGM12);
  TCCR1B |= (1 << CS12) | (1 << CS10);  
  TIMSK1 |= (1 << OCIE1A);

  sei();

  Serial.begin(38400);
}

int16_t * get_re(int loc){
  switch(loc){
    case 0:
      return re0;
    case 1:
      return re1;
    default:
      return re0;
  }
}


void loop(void) {
  if (re_pointer%AVG_SIZE == 0 && re_pointer <= SIZE){ // TODO: REMOVE second condition if not work
    write_arr = !write_arr;
    envelope(get_re(!write_arr), out, re_pointer>>AVG_SHIFT);
  }
  if (re_pointer == (int) (SIZE / 3)) {
    digitalWrite(TXLED, LOW);
  }
  if (re_pointer == (int) (SIZE * 2 / 3)) {
    digitalWrite(RXLED, LOW);
  }
  if (re_pointer == SIZE) {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(TXLED, HIGH);
    digitalWrite(RXLED, HIGH);

    // Print out recorded data
    Serial.println("Start");
    for (int i = 0; i < SIZE_AFTER_FILTER; i++) {
      Serial.println(out[i]);
    }

    delay(2000);
    re_pointer = 0;
  }
}


/*---------------------------*/
/*    Interrupt functions    */
/*---------------------------*/

ISR(TIMER1_COMPA_vect){
  if (re_pointer < SIZE) {
    digitalWrite(LED_BUILTIN, HIGH);
    get_re(write_arr)[re_pointer%AVG_SIZE] = (analogRead(MIC_INPUT) >> 4) - 128;
    re_pointer += 1;

  }
}
