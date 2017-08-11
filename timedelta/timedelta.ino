unsigned long last1 = millis();
long last2 = millis();
bool sense1;
bool sense2;
long prev = 0;
\
void setup()
{
  Serial.begin(9600);
  pinMode(23, OUTPUT);
  digitalWrite(23,HIGH);
}
int counter = 0;

void loop()  {
  Serial.print(digitalRead(53));
  Serial.println(digitalRead(52));
}

