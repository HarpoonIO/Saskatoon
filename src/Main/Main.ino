#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "Servo.h"
#include "ESC_library.h"

int recieverThrottle = 10;
int tempThrottle = 0;
int throttle = 0;
int motor2Throttle = 1100;
int motor4Throttle = 1100;
int motor1Throttle = 0;
int motor3Throttle = 0;

// ESC's
int motor3 = 3; // roll
int motor2 = 4; // pitch
int motor1 = 5; // roll
int motor4 = 6; // pitch
ESC_library esc(motor1, motor2, motor3, motor4);

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// indicates whether MPU interrupt pin has gone high
volatile bool mpuInterrupt = false;     

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    // Serial.begin(115200);
    // Serial.println("Initializing setup");
    
    // Calibrate ESC's
    esc.calibrate(); 
    pinMode(13, OUTPUT);
    // join I2C bus (I2Cdev library doesn't do this automatically) 
    Wire.begin();
    TWBR = 24; 
    
    // MPU
    // initialize device
    mpu.initialize();
    // verify connection
    mpu.testConnection();
    
    devStatus = mpu.dmpInitialize(); // if != 0 -> then something is wrong!
    digitalWrite(13, 1);
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);
        // enable Arduino interrupt detection
        // 0 -> because pin 2 is interrupt 0 on mega
        // RISING: triggers interrupt when pin goes from low -> high
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;
        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();  
        // Serial.println("MPU6050 connection successful");
    } else {
        // ERROR!
        // Serial.println("Couldn't connect the MPU6050");
    }
    
    // Serial.println("Setup finished!");
    
    // For PID
    setGains(0.60,0.5,1);
    setGainsRoll(0.60,0.5,1);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // Other code goes here!
        // listenForController();
        // controlThrottle();
        // computePIDpitch(); // register for change in pitch
        // controlPitchThrottle();
        computePIDroll();     // register for change in roll
        controlRollThrottle();
    }

    checkForDofData();
    
}
// ================================================================
// ===                        FUNCTIONS                         ===
// ================================================================

/**
 * Listens for input from receiver and changes
 * the throttle if the user changes
 * power on Channel3
 */
void listenForController(){
  //tempThrottle = map(analogRead(recieverThrottle), 0, 1023, 700, 2000);
  tempThrottle = map(pulseIn(recieverThrottle, HIGH), 996, 2000, 0, 1300);
  //Serial.print("From receiver: ");
  //Serial.println(pulseIn(10, HIGH));
  throttle = tempThrottle;
}

/**
 * Controls the overall speed on the 4 motors
 */
void controlThrottle(){
  esc.controlMotor1(780);
  //esc.controlMotor2(motor2Throttle);
  // esc.controlMotor3(780);
  //esc.controlMotor4(motor4Throttle);
}

/**
 * Checks whether the MPU's FIFO que has data or not.
 * If it has data greater than packetSize, it calls 
 * an interrupt for the arduino.
 */
void checkForDofData(){
  // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (if this happens, our code isn't good enough)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
    // otherwise, check for DMP data ready interrupt (hopefully this happens very often)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        
        // Serial.print("ypr\t");
        // Serial.print(ypr[0] * 180/M_PI);
        // Serial.println("\t");
        // Serial.println(ypr[1] * 180/M_PI);
        // Serial.print("\t");
        // Serial.println(ypr[2]);
        
        
    }
}

/**
 * This is our Interrupt Service Routine (ISR)
 * Has to be a no-argument function
 * Will be called when there's an interrupt
 */
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                TEMP FUNCTIONS FOR TESTING                ===
// ================================================================

// PID for PITCH (y - axis)
unsigned long lastTimePitch;
double processVariablePitch = ypr[1], pitchAxisOutput, setPointPitch = -0.055; // Correct: -0.045 or -0.025
double errSumPitch, lastErrPitch;
float kp, ki, kd;
double dErr;

void computePIDpitch(){
    processVariablePitch = ypr[1];
    /*How long since we last calculated*/
    unsigned long now = micros();            
    double timeChange = (double) (now - lastTimePitch);
      
    /*Compute all the working error variables*/
    double error = setPointPitch - processVariablePitch;
    // Don't use the integral before 30 seconds in

    //errSumPitch +=  (error * (timeChange / 100000));
    errSumPitch = ((errSumPitch * 0.999) + (error * 0.001));
    
    
    if(error != lastErrPitch){
        dErr = (100000.0 * (error - lastErrPitch)) / timeChange;  
        /*Remember some variables for next time*/
        lastErrPitch = error;
        lastTimePitch = now;
    }
    // Serial.print("Algorithm: ");
    // Serial.println(pitchAxisOutput, 3);

    /*Compute PID Output*/
    pitchAxisOutput = (kp * error) + (ki * errSumPitch) + (kd * dErr);// (kp * error) + (ki * errSumPitch) ; // + (kd * dErr)

}

/**
 * Gives throttle for motor 2 and 4 - PITCH
 */
int oldPitchThrottleMotor2, oldPitchThrottleMotor4;
void controlPitchThrottle() {   
    double newPitchThrottleMotor2 = map((pitchAxisOutput * 1000), -1000, 1000, 700, 2000);
    double newPitchThrottleMotor4 = map((pitchAxisOutput * 1000), 1000, -1000, 700, 2000);
    // Serial.print("Motor 2 throttle: ");
    // Serial.println(newPitchThrottleMotor2);
    // Serial.print("Motor 4 throttle: ");
    // Serial.println(newPitchThrottleMotor4);
    
    // Serial.print("PID contrller: ");
    // Serial.println(pitchAxisOutput);
    if(newPitchThrottleMotor2 < 700){
        newPitchThrottleMotor2 = 700;
    }

    if(newPitchThrottleMotor2 > 2000){
        newPitchThrottleMotor2 = 2000;
    }
    // motor 2 needs power
    esc.controlMotor2(newPitchThrottleMotor2); 

    if(newPitchThrottleMotor4 > 2000){
        newPitchThrottleMotor4 = 2000;
    }

    if(newPitchThrottleMotor4 < 700){
        newPitchThrottleMotor4 = 700;
    }
    // motor 4 needs power
    esc.controlMotor4(newPitchThrottleMotor4);
    // Update variable(s)
    oldPitchThrottleMotor2 = newPitchThrottleMotor2;   
    oldPitchThrottleMotor4 = newPitchThrottleMotor4;
    
} 

// -------------------------------------------------------------------
// PID for ROLL (x - axis)
unsigned long lastTimeRoll;
double processVariableRoll = ypr[2], rollAxisOutput, setPointRoll = -0.004; 
double errSumRoll, lastErrRoll;
double dErrRoll;
float kpRoll, kiRoll, kdRoll;

void computePIDroll(){
    processVariableRoll = ypr[2];
    /*How long since we last calculated*/
    unsigned long now = micros();            
    double timeChange = (double) (now - lastTimeRoll);
      
    /*Compute all the working error variables*/
    double error = setPointRoll - processVariableRoll;

    // TODO: Don't use the integral before 30 seconds in
    //errSumPitch +=  (error * (timeChange / 100000));
    errSumRoll = ((errSumRoll * 0.999) + (error * 0.001));
    
    
    if(error != lastErrRoll){
        dErrRoll = (100000.0 * (error - lastErrRoll)) / timeChange; // division by 0!   
        /*Remember some variables for next time*/
        lastErrRoll = error;
        lastTimeRoll = now;
    }

    /*Compute PID Output*/
    rollAxisOutput = (kpRoll * error) + (kiRoll * errSumRoll) + (kdRoll * dErrRoll);
}

/**
 * Gives throttle for motor 1 and 3 - ROLL
 */
void controlRollThrottle(){
    double rollThrottleMotor1 = map((rollAxisOutput * 1000), 1000, -1000, 700, 2000);
    double rollThrottleMotor3 = map((rollAxisOutput * 1000), -1000, 1000, 700, 2000);
    // Serial.print("Motor 1 throttle: ");
    // Serial.println(rollThrottleMotor1);
    // Serial.print("Motor 3 throttle: ");
    // Serial.println(rollThrottleMotor3);
    
    // Serial.print("PID controller: ");
    // Serial.println(rollAxisOutput);
    
    if(rollThrottleMotor3 < 700){
        rollThrottleMotor3 = 700;
    }

    if(rollThrottleMotor3 > 2000){
        rollThrottleMotor3 = 2000;
    }
    
    if(rollThrottleMotor1 > 2000){
        rollThrottleMotor1 = 2000;
    }

    if(rollThrottleMotor1 < 700){
        rollThrottleMotor1 = 700;
    }

    // motor 1 needs power
    esc.controlMotor1(rollThrottleMotor1); 
    // motor 3 needs power
    esc.controlMotor3(rollThrottleMotor3);
}

/**
 * Sets k values for pitch PID algorithm
 */
void setGains(double Kp, double Ki, double Kd){
     kp = Kp;
     ki = Ki;
     kd = Kd;
}

/**
 * Sets k values for roll PID algortihm
 */
void setGainsRoll(double Kp, double Ki, double Kd){
    kpRoll = Kp;
    kiRoll = Ki;
    kdRoll = Kd;   
}







