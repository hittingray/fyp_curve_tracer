// Amplification factor on opamp is 9.2 (1 + 82/10)
// 3200/4095 * 3.3 * 9.2 = 23.7 V 
#define MAXDAC 3100
#define MAXBASE 750
#define LOWVAL 10

// Debounce time in milliseconds
#define DBTIME 300

// Max ADC reading for temperature monitoring
#define MAXTEMP 6700
// 1446 Ohms at 100
// 2450 Ohms at 80

int led = 13;
volatile int shunt_pos, shunt_neg, vb, vc, ve, vbra;
volatile bool state_change = false;
volatile bool rs = false; // True means run!
volatile bool single = false; // 

int sp_pin = A0;
int sn_pin = A1;
int vb_pin = A4;
int vc_pin = A5;
int ve_pin = A6;
int temp_pin = A7;
int vbra_pin = A8;

int avg_runs = 20;


// ---DAC---
// A21 = DAC0 (base)
// A22 = DAC1 (collector)

// ---ADC---
// A0 (14) S+
// A1 (15) S-
// A4 (16) Vb
// A5 (17) Vc
// A6 (18) Ve
// A7 (19) Temperature

// ---Switch---
// 11 PNP/NPN -- 0 is NPN, 1 is PNP -- !!!Actually analogRead(A4)!!!
// 12 Vb/Vc -- 0 is Vb, 1 is Vc
// 24 Run/Stop
// 25 Single

// ---Output---
// 9 Green LED
// 10 Red LED

// "++" indicates start of new trace
// "--" indicates end of trace

// "M1" indicates Ic vs Vbe
// "M2" indicates Ic vs Vce

unsigned long liState = 0; // li = last interrupt
void state() {
  if (millis() - liState > DBTIME) {
    liState = millis();
    state_change = true;
    // This will break any currently running sweep and restart it
  }
}

unsigned long liRS = 0; // li = last interrupt
void run_stop() {
  if (millis() - liRS > DBTIME) {
    liRS = millis();
    rs = !rs;
    state_change = true;
    // Will cause the sweep to prematurely end and pause there
  
    // LEDs for run status
    if (rs) {
      digitalWrite(10, LOW);
      digitalWrite(9, HIGH);
    }
    else {
      digitalWrite(9, LOW);
      digitalWrite(10, HIGH);
    }
  }

}

unsigned long liSingle = 0; // li = last interrupt
void single_fire() {
  if (millis() - liSingle > DBTIME) {
    liSingle = millis();
    state_change = true;
    single = true;
    rs = true;
    // Will start a new single fire run
  
    // LEDs for run status
    digitalWrite(10, LOW);
    digitalWrite(9, HIGH);
  }
}

void setup() {
  digitalWrite(led, HIGH);
  analogWriteResolution(12);
  analogReadResolution(13);
  

  pinMode(11, INPUT);
  pinMode(12, INPUT_PULLUP);
  pinMode(24, INPUT_PULLUP);
  pinMode(25, INPUT_PULLUP);

  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  
  digitalWrite(10, HIGH);
  digitalWrite(9, LOW);

  attachInterrupt(digitalPinToInterrupt(11), state, CHANGE);
  attachInterrupt(digitalPinToInterrupt(12), state, CHANGE);
  attachInterrupt(digitalPinToInterrupt(24), run_stop, FALLING);
  attachInterrupt(digitalPinToInterrupt(25), single_fire, FALLING);

  Serial.begin(2000000);
  while (!Serial) {
    if (analogRead(ve_pin) < 4095) {
      analogWrite(A21, LOWVAL);
      analogWrite(A22, LOWVAL);
    }
    else {
      analogWrite(A21, MAXDAC + 200);
      analogWrite(A22, MAXDAC + 200);
    }
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
    delay(500);
    digitalWrite(10, LOW);
    digitalWrite(9, HIGH);
    delay(500);
    // wait for serial port to connect. Needed for native USB
}
}

void serial_readsend(int vbr) {
  if (analogRead(temp_pin) > MAXTEMP) {
    state_change = true;
  }
  
  shunt_pos = avg_read(sp_pin, avg_runs);
  shunt_neg = avg_read(sn_pin, avg_runs);
  vb = avg_read(vb_pin, avg_runs);
  vc = avg_read(vc_pin, avg_runs);
  ve = avg_read(ve_pin, avg_runs);
  vbra = avg_read(vbra_pin, avg_runs);

  Serial.print(shunt_pos);
  Serial.print(",");
  Serial.print(shunt_neg);
  Serial.print(",");
  Serial.print(vb);
  Serial.print(",");
  Serial.print(vc);
  Serial.print(",");
  Serial.print(ve);
  Serial.print(",");
  Serial.print(vbr); // This is the value of the DAC into the base
  Serial.print(",");
  Serial.print(vbra); // This is the value of the DAC into the base
  Serial.println();
}

int avg_read(int pin, int num_runs) {
  float value = 0;
  float avg_untouched;
  float arr[num_runs];
  float num_runs_minus_outliers = num_runs;

  // Add to array, add to current sum
  for (int i = 0; i < num_runs; i++) {
    arr[i] = (float) analogRead(pin);
    value += arr[i];
  }

  // Find average
  avg_untouched = value / num_runs;

  // Go through array and look for outliers greater than 40% different from average. Remove them from the sum;
  // Calculating IQR and taking values between Q1 and Q3 would be ideal, but that takes a long time and isn't necessary for the rough measurements here
  for (int i = 0; i < num_runs; i++) {
    if (fabs((avg_untouched - arr[i])/avg_untouched) > 0.4) {
      value -= arr[i];
      num_runs_minus_outliers--;
    }
  }

  value /= num_runs_minus_outliers;
  return (int) value;
}

void loop() {
  while (!rs) {
    if (analogRead(ve_pin) < 4095) {
      analogWrite(A21, 0);
      analogWrite(A22, 0);
    }
    else {
      analogWrite(A21, MAXDAC + 200);
      analogWrite(A22, MAXDAC + 200);
    }
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
    // Trap for if run is false
  }

  if (rs) {
    digitalWrite(10, LOW);
    digitalWrite(9, HIGH);
  }
  
  // Set the state change flag to false at start of each sweep
  state_change = false;
  Serial.println("++");
  
  //  NPN
  if (analogRead(ve_pin) < 4095) {
    Serial.println("NPN");
    // Vb
    if (digitalRead(12) == 0) {
      Serial.println("M1");
      analogWrite(A22, MAXDAC/2);
      for (int i = LOWVAL; i < MAXBASE; i += 1) {
        analogWrite(A21, i);
        serial_readsend(i);
        if (state_change) break;
      }
      analogWrite(A21, LOWVAL);
      analogWrite(A22, LOWVAL);
    }
    // Vc
    else {
      Serial.println("M2");
      for (int i = LOWVAL; i < MAXBASE; i += 40) {
        analogWrite(A21, i);
        for (int j = LOWVAL; j < MAXDAC; j += 3) {
            analogWrite(A22, j);
            serial_readsend(i);
            if (state_change) break;
        }
        analogWrite(A21, LOWVAL);
        analogWrite(A22, LOWVAL);
        delay(5);
        if (state_change) break;
      }
    }
  }

  // PNP
  else {
    Serial.println("PNP");
    // Vb
    if (digitalRead(12) == 0) {
      Serial.println("M1");
      analogWrite(A22, MAXDAC/2);
      for (int i = MAXDAC; i > MAXDAC - MAXBASE; i -= 1) {
        analogWrite(A21, i);
        serial_readsend(i);
        if (state_change) break;
      }
      analogWrite(A21, MAXDAC);
      analogWrite(A22, MAXDAC);
    }
    // Vc
    else {
      Serial.println("M2");
      for (int i = MAXDAC; i > MAXDAC - MAXBASE; i -= 40) {
        analogWrite(A21, i);
        for (int j = MAXDAC; j > LOWVAL; j -= 3) {
            analogWrite(A22, j);
            serial_readsend(i);
            if (state_change) break;
        }
        analogWrite(A21, MAXDAC);
        analogWrite(A22, MAXDAC);
        delay(5);
        if (state_change) break;
      }
    }
  }
  Serial.println("--");

  // If we press the single button again in the middle of a run, it should restart, not stop
  if (single && !state_change) {
    rs = false;
    single = false;
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
  }

  if (analogRead(temp_pin) > MAXTEMP) {
    rs = false;
    single = false;
    digitalWrite(9, LOW);
    digitalWrite(10, HIGH);
    while (analogRead(temp_pin) > MAXTEMP) {
      Serial.println("OV");
    }
    Serial.println("CD");
  }
  
}
