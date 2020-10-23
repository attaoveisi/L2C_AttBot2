
// Please insert your motor encoder output pulse per rotation
#define ENCODEROUTPUT 12

#define HALLSEN_FR_A 2 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)
#define HALLSEN_FL_A 3 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)
#define HALLSEN_RR_A 4 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)
#define HALLSEN_RL_A 5 // Hall sensor A of front right wheel connected to pin A15 (external interrupt)

volatile long encoderValue_FR = 0;
volatile long encoderValue_FL = 0;
volatile long encoderValue_RR = 0;
volatile long encoderValue_RL = 0;

int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

double rpm_FR = 0.0;
double rpm_FL = 0.0;
double rpm_RR = 0.0;
double rpm_RL = 0.0;
double vx_FR = 0.0;
double vx_FL = 0.0;
double vx_RR = 0.0;
double vx_RL = 0.0;
double v_right = 0.0;
double v_left = 0.0;
double vx = 0.0;
double vy = 0.0;
double vth = 0.0;
double dt = 0.0;
double delta_x = 0.0;
double delta_y = 0.0;
double delta_th = 0.0;
double x_driven = 0.0;
double y_driven = 0.0;
double th_driven = 0.0;
double lengthBetweenTwoWheels = 130.0/1000.0; //130 mm

void updateEncoder_FR()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_FR++;
}

void updateEncoder_FL()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_FL++;
}

void updateEncoder_RR()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_RR++;
}

void updateEncoder_RL()
{
  // Add encoderValue by 1, each time it detects rising signal
  // from hall sensor A
  encoderValue_RL++;
}

void setup()
{
  Serial.begin(57600); // Initialize serial with 9600 baudrate

  pinMode(HALLSEN_FR_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_FL_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_RR_A, INPUT_PULLUP); // Set hall sensor A as input pullup
  pinMode(HALLSEN_RL_A, INPUT_PULLUP); // Set hall sensor A as input pullup

  // Attach interrupt at hall sensor A on each rising signal
  attachInterrupt(digitalPinToInterrupt(HALLSEN_FR_A), updateEncoder_FR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_FL_A), updateEncoder_FL, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_RR_A), updateEncoder_RR, RISING);
  attachInterrupt(digitalPinToInterrupt(HALLSEN_RL_A), updateEncoder_RL, RISING);
}

void loop()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {
      // Revolutions per minute (RPM) = (total encoder pulse in 1s / motor encoder output) x 60s
      rpm_FR = (double)(encoderValue_FR * 60 / ENCODEROUTPUT);
      rpm_FL = (double)(encoderValue_FL * 60 / ENCODEROUTPUT);
      rpm_RR = (double)(encoderValue_RR * 60 / ENCODEROUTPUT);
      rpm_RL = (double)(encoderValue_RL * 60 / ENCODEROUTPUT);
      vx_FR = abs(rpm_FR*0.003401667);
      vx_FL = abs(rpm_FL*0.003401667);
      vx_RR = abs(rpm_RR*0.003401667);
      vx_RL = abs(rpm_RL*0.003401667);
      v_right = (vx_FR+vx_RR)/2.0;
      v_left = (vx_FL+vx_RL)/2.0;
      vx = (v_right + v_left)/2.0;
      vy = 0;
      vth = ((v_right - v_left)/lengthBetweenTwoWheels);
      dt = currentMillis - previousMillis;
      delta_x = (vx * cos(th_driven)) * dt/1000.0;
      delta_y = (vx * sin(th_driven)) * dt/1000.0;
      delta_th = vth * dt/1000.0;
      x_driven += delta_x;
      y_driven += delta_y;
      th_driven += delta_th;
      while (th_driven > PI) {
        th_driven -= 2.0 * PI;
      }
      while (th_driven < -PI) {
        th_driven += 2.0 * PI;
      }
    
      previousMillis = currentMillis;
      // Reset the encoders 
      encoderValue_FR = 0;
      encoderValue_FL = 0;
      encoderValue_RR = 0;
      encoderValue_RL = 0;
    }

    Serial.print('\t');
    Serial.print("rpFR");
    Serial.print('\t');
    Serial.print("rpFL");
    Serial.print('\t');
    Serial.print("rpRR");
    Serial.print('\t');
    Serial.print("rpRL");
    Serial.print('\t');
    Serial.print("vxFR");
    Serial.print('\t');
    Serial.print("vxFL");
    Serial.print('\t');
    Serial.print("vxRR");
    Serial.print('\t');
    Serial.print("vxRL");
    Serial.print('\t');
    Serial.print("vrit");
    Serial.print('\t');
    Serial.print("vlt");
    Serial.print('\t');
    Serial.print("v_x");
    Serial.print('\t');
    Serial.print("v_y");
    Serial.print('\t');
    Serial.print("v_th");
    Serial.print('\t');
    Serial.print("dt1");
    Serial.print('\t');
    Serial.print("delx");
    Serial.print('\t');
    Serial.print("dely");
    Serial.print('\t');
    Serial.print("delt");
    Serial.print('\t');
    Serial.print("xven");
    Serial.print('\t');
    Serial.print("yven");
    Serial.print('\t');
    Serial.println("tven"); 
    
    Serial.print('\t');
    Serial.print(rpm_FR);
    Serial.print('\t');
    Serial.print(rpm_FL);
    Serial.print('\t');
    Serial.print(rpm_RR);
    Serial.print('\t');
    Serial.print(rpm_RL);
    Serial.print('\t');
    Serial.print(vx_FR);
    Serial.print('\t');
    Serial.print(vx_FL);
    Serial.print('\t');
    Serial.print(vx_RR);
    Serial.print('\t');
    Serial.print(vx_RL);
    Serial.print('\t');
    Serial.print(v_right);
    Serial.print('\t');
    Serial.print(v_left);
    Serial.print('\t');
    Serial.print(vx);
    Serial.print('\t');
    Serial.print(vy);
    Serial.print('\t');
    Serial.print(vth);
    Serial.print('\t');
    Serial.print(dt);
    Serial.print('\t');
    Serial.print(delta_x);
    Serial.print('\t');
    Serial.print(delta_y);
    Serial.print('\t');
    Serial.print(delta_th);
    Serial.print('\t');
    Serial.print(x_driven);
    Serial.print('\t');
    Serial.print(y_driven);
    Serial.print('\t');
    Serial.println(th_driven); 
}
