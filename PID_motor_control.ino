#define EncoderA 2
#define EncoderB 3
#define out 5
#define pin1 13
#define pin2 12
int pulses = 0; //for counting pulses
float RPM0 = 0; //for holding previous RPM reading
float RPM1 = 0; //for holding current RPM reading
float IE = 0; //integral error
double time0 = 0; //previous time
double time1 = 0; //current time
float perror0 = 0, perror1 =0; //current and previous proportional errors
float u0 = 0; //stores the previous forecast
class PID{
  public:
    int sp; //setpoint: the value in RPM we want to acheive
    float Kp = 0, Ki = 0, Kd = 0; //PID parameters
    PID(float Kp, float Ki, float Kd, int sp): Kp(Kp), Ki(Ki), Kd(Kd), sp(sp){}
    float proportional() //proportional term
    {
      float error = sp - RPM1; //getting error
      return (Kp*error);
    }
    float derivative(float error0, float error1) //derivative term
    {
      float slope = (error1 - error0) / (time1 - time0);
      float ferror = Kd*slope;
      return ferror;
    }
    float integral(float error) //integral term, takes proportional error as argument
    {
      IE = IE + (error / Kp)*(time1 - time0);
      return IE;
    }
};

PID controller1(0,0,0, 10);
void setup() {
  pinMode(EncoderA, INPUT);
  pinMode(EncoderB, INPUT);
  pinMode(out, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(EncoderA), count, RISING); //configuring an interupt pin in order that calls the count function on a pulse from the encoder
}

void loop() {
  float perror1 = controller1.proportional(); //storing current proportional error in a variable
  float u1 = perror1 + controller1.derivative(perror0, perror1) + controller1.integral(perror1); //adding the outputs of the PID functions
  perror0 = perror1; //the current error becomes the previous error after its used
  u1 = smooth(u1);
  u0 = u1;
  motor_control(map(u1, 0, 130, 0, 255)); //we send the output to a function that controls the motor, mapped from the min and max RPM to the min and max PWM values
}

void count(){
  if(digitalRead(EncoderB)) //if encoderB is outputing 1, then we increase a pulse
    pulses++;
  else //else, we are going in the opposite direction
    pulses--;
  if (pulses == 540) //after each revolution, we calculate the RPM
  {
    pulses = 0;
    RPM0 = RPM1;
    time1 = millis();
    RPM1 = 1 / (time1 - time0);
    time0 = time1;
  }
}

void motor_control(long u)
{
  analogWrite(out, u); //writing the output to the pwm pin
  if(controller1.sp > 0)
  {
    digitalWrite(pin1, HIGH);
    digitalWrite(pin2, LOW);
  }
  else if(controller1.sp < 0)
  {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, HIGH);
  }
  else
  {
    digitalWrite(pin1, LOW);
    digitalWrite(pin2, LOW);
  }
}

float smooth(float u1) //F1 = aA + (1 - a)F0
{
  float a = 0.2; //exponential smoothing factor
  float F = a*u1 + (1 - a)*u0;
  return F;
}