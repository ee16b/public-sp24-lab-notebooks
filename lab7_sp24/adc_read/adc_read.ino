int res;
#define RED_LED                     17
#define GREEN_LED                   30
#define READ_PIN                    A2

void setup()
{
  pinMode(READ_PIN, INPUT);

/*
 * In spring 2018 we attempted changing the baud rate to 38400 to match
 * the rest of the project, but it did not work.
 * KEEP AT 9600.
 */
  Serial.begin(9600);
}

void loop()
{
  delay(2);
  res = analogRead(READ_PIN);
  //split into 5 bit chunks
  Serial.write((byte) ((res >> 6) & 63) | 64);
  Serial.write((byte) (res & 63));
}
