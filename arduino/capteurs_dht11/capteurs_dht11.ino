/*
  capteurs_dht11.ino - Arduino Uno fixe sur la tete du Yanshee

  Reproduit le programme d'origine (code source perdu, comportement retrouve en
  desassemblant arduino/sauvegarde_origine.hex) et ajoute le DHT11.

  Envoie en continu, a 115200 bauds, une ligne au meme format qu'avant :
      c=28 g=-1 d=-1 gaz=64 vapeur=26 lum=50 temp=24 hum=55
  -> capteurs_serveur.py la lit telle quelle, rien a changer sur le robot.

  Broches (identiques a l'original) :
    HC-SR04 torse : Trig D9, Echo D10   -> c (cm, -1 si pas d'echo ou > 400 cm)
    g et d        : toujours -1 (l'original ne gerait qu'un ultrason)
    MQ-2 gaz      : A0                  -> gaz
    Thermistance  : A1 (NTC 10k, B 3950) -> tadc, temp_ntc  (rien de branche aujourd'hui)
    Vapeur        : A2                  -> vapeur
    Lumiere       : A3                  -> lum
  Nouveau :
    DHT11         : DATA sur D2         -> temp (degres C), hum (%)

  Comme l'original, une entree analogique n'est envoyee que si un capteur y est
  branche (test avec la resistance de tirage interne).
  Dans l'original, la thermistance s'appelait "temp" ; ici "temp" est le DHT11
  et la thermistance devient "temp_ntc".

  Bibliotheque a installer (IDE Arduino > Outils > Gerer les bibliotheques) :
    "DHT sensor library" d'Adafruit (accepter aussi "Adafruit Unified Sensor").

  Pour remettre le programme d'origine : voir arduino/sauvegarde_origine.hex.
*/

#include <DHT.h>
#include <math.h>

const int TRIG = 9;
const int ECHO = 10;
const int PIN_GAZ = A0;
const int PIN_NTC = A1;
const int PIN_VAPEUR = A2;
const int PIN_LUM = A3;
const int PIN_DHT = 2;

const unsigned long TIMEOUT_ECHO_US = 25000;   // ~4 m, comme l'original
const unsigned long PERIODE_DHT_MS = 2000;     // le DHT11 ne supporte pas plus de 1 lecture/s

DHT dht(PIN_DHT, DHT11);
float temperature = NAN;
float humidite = NAN;
unsigned long dernierDht = 0;

void setup() {
  Serial.begin(115200);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  digitalWrite(TRIG, LOW);
  dht.begin();
}

// Distance en cm, -1 si pas d'echo ou au-dela de 400 cm.
long distanceCm() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  unsigned long duree = pulseIn(ECHO, HIGH, TIMEOUT_ECHO_US);
  if (duree == 0) return -1;
  long cm = duree / 58;
  if (cm > 400) return -1;
  return cm;
}

// Capteur branche ? Avec la resistance de tirage, une entree en l'air monte a ~1023.
bool present(int pin) {
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(300);
  analogRead(pin);
  int v = analogRead(pin);
  pinMode(pin, INPUT);
  delayMicroseconds(300);
  return v < 1005;
}

// Moyenne de 8 lectures (la premiere, souvent faussee, est jetee).
int lireMoyenne(int pin) {
  analogRead(pin);
  long somme = 0;
  for (int i = 0; i < 8; i++) somme += analogRead(pin);
  return somme / 8;
}

void lireDht() {
  if (millis() - dernierDht < PERIODE_DHT_MS) return;
  dernierDht = millis();
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (!isnan(h) && !isnan(t)) {       // en cas d'echec, on garde la derniere bonne valeur
    humidite = h;
    temperature = t;
  }
}

void envoyerAnalogique(const char *cle, int pin) {
  if (!present(pin)) return;
  Serial.print(cle);
  Serial.print(lireMoyenne(pin));
}

void envoyerThermistance() {
  if (!present(PIN_NTC)) return;
  int adc = lireMoyenne(PIN_NTC);
  Serial.print(" tadc=");
  Serial.print(adc);
  if (adc < 3 || adc > 1000) return;
  float r = 10000.0 * (1023.0 / adc - 1.0);
  float t = 1.0 / (log(r / 10000.0) / 3950.0 + 1.0 / 298.15) - 273.15;
  if (t > -20 && t < 80) {
    Serial.print(" temp_ntc=");
    Serial.print(t, 1);
  }
}

void loop() {
  long c = distanceCm();
  lireDht();

  Serial.print("c=");
  Serial.print(c);
  Serial.print(" g=-1 d=-1");
  envoyerAnalogique(" gaz=", PIN_GAZ);
  envoyerThermistance();
  envoyerAnalogique(" vapeur=", PIN_VAPEUR);
  envoyerAnalogique(" lum=", PIN_LUM);
  if (!isnan(temperature)) {
    Serial.print(" temp=");
    Serial.print(temperature, 0);
    Serial.print(" hum=");
    Serial.print(humidite, 0);
  }
  Serial.println();
  delay(20);
}
