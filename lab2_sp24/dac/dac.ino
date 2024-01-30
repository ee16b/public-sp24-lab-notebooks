/* Set the serial connection speed */
#define BAUD_RATE       9600

/* Set which bits correspond to which pins */
#define BIT_3          	7
#define BIT_2           8
#define BIT_1           12
#define BIT_0           13

/* Set number of bits */
#define BITS            3

/* Set how long to delay in between digital outputs */
#define DELAY           10


int val = 0;
int added = 1;

void setup()
{
  // Start a serial connection to send data to the computer
  Serial.begin(BAUD_RATE);

  // Set pins to output mode
  pinMode(BIT_3, OUTPUT);
  pinMode(BIT_2, OUTPUT);
  pinMode(BIT_1, OUTPUT);
  pinMode(BIT_0, OUTPUT);
}

void loop()
{
  // If LSB is set in val, turn bit0 high
  if (val & 1)
    digitalWrite(BIT_0, HIGH);
  else
    digitalWrite(BIT_0, LOW);

  // If bit 1 is set in val, turn bit1 high
  if (val & 2)
    digitalWrite(BIT_1, HIGH);
  else
    digitalWrite(BIT_1, LOW);

  // If bit 2 is set in val, turn bit2 high
  if (val & 4)
    digitalWrite(BIT_2, HIGH);
  else
    digitalWrite(BIT_2, LOW);


  if (BITS == 4){
    // If bit 3 is set in val, turn bit3 high
    if(val & 8)
    digitalWrite(BIT_3, HIGH);
    else
    digitalWrite(BIT_3, LOW);

    // Range of values is 0 to 15 for 4 bit DAC
    if(val == 15)
    added = -1;
  }


  if (BITS == 3){
    // Range of values is 0 to 7 for 3 bit DAC
    if (val == 7)
      added = -1;
  }


  if (val == 0)
    added = 1;

  val = val + added;

  delay(DELAY);

}
