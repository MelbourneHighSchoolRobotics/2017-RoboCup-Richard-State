unsigned long last1 = millis();
long last2 = millis();
bool sense1;
bool sense2;
long prev = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(8, OUTPUT);
}
int counter = 0;

void loop()  {
  //Serial.println(digitalRead(8));
  Serial.println(analogRead(A9));
}

