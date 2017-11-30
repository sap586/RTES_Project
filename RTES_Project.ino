#define right_echo PE11         //PE11
#define right_trigger PE10      //PE10
#define front_echo PF6          //PF6
#define front_trigger PA12      //PA12  - PD0
#define left_echo PA13          //PA13  - PF9
#define left_trigger PA11       //PA11  - PB0
float distance_r, distance_f, distance_l;
float ultrasonic(int trig_pin, int echo_pin);

static int const motor1_PD6 = PD6;
static int const motor1_PD7 = PD7;
static int const motor2_PD3 = PD3;
static int const motor2_PD4 = PD4;
//MOTOR STOP FLAG
static int const motor_stop_flag = PE8;
static int const motor_turn_flag = PE9;

static int const encoder_pin_right = PA1;
static int const encoder_pin_left = PA2;
volatile int pulses_right = 0;
volatile int pulses_left = 0;
unsigned int pulsesperturn = 20;
void forward_left (void);
void motor_forward (float duty);
void motor_turn_180(void);
void align_r (void);
void align_l (void);
void wall (void);
float pulse_in(uint8_t pin, uint8_t state);

void motor(int pin, float duty);


void setup() {
  Serial1.begin(9600);
  pinMode(right_echo, INPUT);
  pinMode(right_trigger, OUTPUT);
  pinMode(front_echo, INPUT);
  pinMode(front_trigger, OUTPUT);
  pinMode(left_echo, INPUT);
  pinMode(left_trigger, OUTPUT);
}

void loop() {
  distance_r = ultrasonic(right_trigger, right_echo);
  distance_f = ultrasonic(front_trigger, front_echo);
//  distance_l = ultrasonic(left_trigger, left_echo);
//  Readings of ultrasonic sensors, can be monitored via serial cable
//  Serial1.print(distance_l);
//  Serial1.print("\t");
//  Serial1.print(distance_f);
//  Serial1.print("\t");
//  Serial1.println(distance_r);

//  The following code takes the garbage values and performs the task accordingly.
//  The main wall following algorithm will be called only when all the garbage values are omitted.
 if(distance_f < 0.5 || distance_f > 500 || distance_r > 500){
  motor(motor2_PD3, 0.4);
  delay(60);
  motor_stop();
 } else {
    wall();
  }

}

//  turning on the trigger pin of HCSR sensor will generate the ultrasound.
//  The echo pin senses back the ultrasound and the distance is returned based on the time.
float ultrasonic(int trig_pin, int echo_pin) {
  float distance;
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  distance = pulse_in(echo_pin, HIGH);
  return distance;
}

// The value of loop_count variable will be returned by the function which can be divided required value to measure the precise distance.
float pulse_in(uint8_t pin, uint8_t state) {
  float pulse_width = 0.00;
  unsigned long loop_count = 0;
  unsigned long loop_max = 5000000;
  while (digitalRead(pin) != state) {
    if (loop_count++ == loop_max) {
      return 0;
    }
  }
  while (digitalRead(pin) == state) {
    if (loop_count++ == loop_max) {
      return 0;
    }
    pulse_width++;
  }
  return pulse_width / 100.00;
}


void motor_stop() {
  motor(motor1_PD6, 0);
  motor(motor1_PD7, 0);
  motor(motor2_PD3, 0);
  motor(motor2_PD4, 0);
}

void motor_forward(float duty) {
  motor(motor1_PD7, duty);
  motor(motor2_PD4, duty);
}

void motor_reverse(float duty) {
  motor(motor1_PD6, duty);
  motor(motor2_PD3, duty);
}

//  PWM in action.
void motor(int pin, float duty) {
  float ccr_val = duty * 1000;
  pinMode(pin, PWM);
  timer_dev *timerDevice = PIN_MAP[pin].timer_device;
  uint8 timerChannel = PIN_MAP[pin].timer_channel;
  timer_pause(timerDevice);
  timer_set_prescaler(timerDevice, 650); //setting prescale
  timer_set_reload(timerDevice, 999); //setting ARR (or period)
  timer_oc_set_mode(timerDevice, timerChannel, TIMER_OC_MODE_PWM_1, TIMER_OC_PE);
  timer_set_compare(timerDevice, timerChannel, ccr_val);
  timer_resume(timerDevice);
}


void setEncoder_right(int encoder_pin_right) {
  pinMode(encoder_pin_right, INPUT);
  attachInterrupt(PA1, counter_right, RISING);
}
void setEncoder_left(int encoder_pin_left) {
  pinMode(encoder_pin_left, INPUT);
  attachInterrupt(PA2, counter_left, RISING);
}

void counter_right() {
  delay(5);
  pulses_right++;
}

void counter_left() {
  delay(5);
  pulses_left++;
}

//  The right turn of the robot is taken precisely by sensing the real time values of the right distance.
//  The robot will keep on turning left untill the distance sensed by the right sensor is under the range.
void forward_right(void) {
  motor_stop();
  delay(200);
  motor_forward(0.4);
  delay(200);
//  setEncoder_right(encoder_pin_right);
  float distance_r2 = distance_r;
  while(distance_r2 > 8){
    motor(motor2_PD4, 0.5);
    motor(motor1_PD6, 0.25);
    delay(50);
    motor_stop();
    distance_r2 = ultrasonic(right_trigger, right_echo);
    motor_forward(0.3);
    delay(55);
  }
//  pulses_left = 0;
  motor_stop();
}

//  The left turn of the robot is also done using the same real time sensing for the front distance.
//  The front distance is constantly kept in consideration and the robot will keep moving left until the front sensor senses the clear path in front of the robot.
void forward_left(){
  motor_stop();
  delay(100);
  float distance_f2 = distance_f;
  while(distance_f2 < 5 || distance_f2 > 500){
    motor(motor2_PD3, 0.35);
    motor(motor1_PD7, 0.55);
    delay(70);
    motor_stop();
    distance_f2 = ultrasonic(front_trigger, front_echo);
  }
}

/*
void forward_left(){
  motor_stop();
  delay(100);
    motor(motor1_PD7, 0.55);
    motor(motor2_PD3, 0.35);
    delay(300);
    motor_stop();
}*/


void motor_turn_180(void) {
  motor_stop();
  delay(150);
  motor(motor2_PD3, 0.6);
  motor(motor1_PD7, 0.6);
  delay(400);
  motor_stop();
}

//  The align functions are called when the robot moves too much away from the right wall or comes too much closer to the right wall.
void align_r (void) {
  motor_stop();
  motor(motor2_PD4, 0.3);
  delay(40);
  motor_stop();
}
void align_l (void) {
  motor_stop();
  motor(motor1_PD7, 0.3);
  delay(40);
  motor_stop();
}
void align_l1 (void) {
  motor_stop();
  motor(motor1_PD7, 0.3);
  delay(50);
  motor_stop();
}

//  The ultimate wall following algorithm that is responsible to get the robot out of the maze following the right wall.
//  The left turn, right turn and U-turn are done with extra care by sensing the relative distances.
void wall (void) {
  if (distance_f < 2 && distance_f > 0.5 || distance_f > 500) {
    float distance_f1 = ultrasonic(front_trigger, front_echo);
    if (distance_f1 < distance_f || distance_f > 500 ) {
      distance_l = ultrasonic(left_trigger, left_echo);
      if (distance_l < 4.5) {
        float distance_l1 = ultrasonic(left_trigger, left_echo);
        if (distance_l1 < distance_l) {
          motor_turn_180();
        } 
      }
      else {
          forward_left();
      }
    }
  }
  else if (distance_f >= 2 && distance_f < 500) {
    if (distance_r <= 2) {
      align_l();
    } else if (distance_r >=200){
      motor(motor2_PD3, 0.4);
      delay(30);
      motor_stop();
    } else if (distance_r >= 3 && distance_r <= 8) {
      align_r();
    }
    else if (distance_r < 3 && distance_r > 2) {
      motor_forward(0.3);
    }
    else if (distance_r > 8) {
      forward_right();
    }
  }
}
