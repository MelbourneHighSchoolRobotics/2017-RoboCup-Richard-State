#define LIGHT_UNLOCK_PIN 22
void setup() {
  Serial.begin(9600);
  pinMode(LIGHT_UNLOCK_PIN,OUTPUT);
  digitalWrite(LIGHT_UNLOCK_PIN,HIGH);
  // put your setup code here, to run once:

}

void loop() {
  CLR0 = analogRead(A0);
  CLR1 = analogRead(A1);
  CLR2 = analogRead(A2);
  Serial.print("A0 :");
  Serial.print(analogRead(A0));
  Serial.print(" A1 :");
  Serial.print(analogRead(A1));
  Serial.print(" A2 :");
  Serial.println(analogRead(A2));
  // put your main code here, to run repeatedly:

}
