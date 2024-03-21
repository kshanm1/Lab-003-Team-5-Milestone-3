#define DEBUG_DRIVE_SPEED    1
#define DEBUG_ENCODER_COUNT  1

#include <Adafruit_NeoPixel.h>

// Function declarations
void Indicator();                                                              // for mode/heartbeat on Smart LED
void setMotor(int dir, int pwm, int in1, int in2);     
void ARDUINO_ISR_ATTR encoderISR(void* arg);

// Encoder structure
struct Encoder {
   const int chanA;                                                            // GPIO pin for encoder channel A
   const int chanB;                                                            // GPIO pin for encoder channel B
   long pos;                                                                   // current encoder position
};

// Port pin constants
#define LEFT_MOTOR_A        35                                                 // GPIO35 pin 28 (J35) Motor 1 A
#define LEFT_MOTOR_B        36                                                 // GPIO36 pin 29 (J36) Motor 1 B
#define RIGHT_MOTOR_A       37                                                 // GPIO37 pin 30 (J37) Motor 2 A
#define RIGHT_MOTOR_B       38                                                 // GPIO38 pin 31 (J38) Motor 2 B
#define ENCODER_LEFT_A      15                                                 // left encoder A signal is connected to pin 8 GPIO15 (J15)
#define ENCODER_LEFT_B      16                                                 // left encoder B signal is connected to pin 8 GPIO16 (J16)
#define ENCODER_RIGHT_A     11                                                 // right encoder A signal is connected to pin 19 GPIO11 (J11)
#define ENCODER_RIGHT_B     12                                                 // right encoder B signal is connected to pin 20 GPIO12 (J12)
#define MODE_BUTTON         0                                                  // GPIO0  pin 27 for Push Button 1
#define MOTOR_ENABLE_SWITCH 3                                                  // DIP Switch S1-1 pulls Digital pin D3 to ground when on, connected to pin 15 GPIO3 (J3)
#define POT_R1              1                                                  // when DIP Switch S1-3 is on, Analog AD0 (pin 39) GPIO1 is connected to Poteniometer R1
#define SMART_LED           21                                                 // when DIP Switch S1-4 is on, Smart LED is connected to pin 23 GPIO21 (J21)
#define SMART_LED_COUNT     1                                                  // number of SMART LEDs in use

// Constants
const int cDisplayUpdate = 100;                                                // update interval for Smart LED in milliseconds
const int cNumMotors = 2;                                                      // number of DC motors
const int cIN1Pin[] = {LEFT_MOTOR_A, RIGHT_MOTOR_A};                           // GPIO pin(s) for INT1
const int cIN1Chan[] = {0, 1};                                                 // PWM channe(s) for INT1
const int c2IN2Pin[] = {LEFT_MOTOR_B, RIGHT_MOTOR_B};                          // GPIO pin(s) for INT2
const int cIN2Chan[] = {2, 3};                                                 // PWM channel(s) for INT2
const int cPWMRes = 8;                                                         // bit resolution for PWM
const int cMinPWM = 150;                                                       // PWM value for minimum speed that turns motor
const int cMaxPWM = pow(2, cPWMRes) - 1;                                       // PWM value for maximum speed
const int cPWMFreq = 20000;                                                    // frequency of PWM signal
const int cCountsRev = 1096;                                                   // encoder pulses per motor revolution

const float wheelDiameter = 4.0;                                               // Diamter of the wheels is 4cm
const float wheelCircumference = 3.14159 * wheelDiameter;                      // Calculate circuimference
const float distancePerRev = wheelCircumference / cCountsRev;                // Distance traveled in one wheel rotation

// Variables
boolean motorsEnabled = true;                                                  // motors enabled flag
boolean timeUp4sec = false; 
boolean timeUp3sec = false;                                                    // 3 second timer elapsed flag
boolean timeUp2sec = false;                                                    // 2 second timer elapsed flag
unsigned char driveSpeed;                                                      // motor drive speed (0-255)
unsigned char driveIndex;                                                      // state index for run mode
unsigned int  modePBDebounce;                                                  // pushbutton debounce timer count
unsigned long timerCount4sec = 0;                                              // 4 second timer count in milliseconds
unsigned long timerCount3sec = 0;                                              // 3 second timer count in milliseconds
unsigned long timerCount2sec = 0;                                              // 2 second timer count in milliseconds
unsigned long displayTime;                                                     // heartbeat LED update timer
unsigned long previousMicros;                                                  // last microsecond count
unsigned long currentMicros;                                                   // current microsecond count

// Declare SK6812 SMART LED object
//   Argument 1 = Number of LEDs (pixels) in use
//   Argument 2 = ESP32 pin number 
//   Argument 3 = Pixel type flags, add together as needed:
//     NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//     NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//     NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//     NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//     NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)
Adafruit_NeoPixel SmartLEDs(SMART_LED_COUNT, SMART_LED, NEO_RGB + NEO_KHZ800);

// smart LED brightness for heartbeat
unsigned char LEDBrightnessIndex = 0; 
unsigned char LEDBrightnessLevels[] = {5,15,30,45,60,75,90,105,120,135,150,165,180,195,210,225,240,255,
                                       240,225,210,195,180,165,150,135,120,105,90,75,60,45,30,15};

unsigned int  robotModeIndex = 0;                                              // robot operational state                              
unsigned int  modeIndicator[6] = {                                             // colours for different modes
   SmartLEDs.Color(255,0,0),                                                   //   red - stop
   SmartLEDs.Color(0,255,0),                                                   //   green - run
}                                                                            

Encoder encoder[] = {{ENCODER_LEFT_A, ENCODER_LEFT_B, 0},                      // left encoder, 0 position 
                     {ENCODER_RIGHT_A, ENCODER_RIGHT_B, 0}};                   // right encoder, 0 position   

void setup()
{
#if defined DEBUG_DRIVE_SPEED || DEBUG_ENCODER_COUNT
   Serial.begin(115200);
#endif

   // Set up motors and encoders
   for (int k = 0; k < cNumMotors; k++) {
      ledcAttachPin(cIN1Pin[k], cIN1Chan[k]);                                  // attach INT1 GPIO to PWM channel
      ledcSetup(cIN1Chan[k], cPWMFreq, cPWMRes);                               // configure PWM channel frequency and resolution
      ledcAttachPin(c2IN2Pin[k], cIN2Chan[k]);                                 // attach INT2 GPIO to PWM channel
      ledcSetup(cIN2Chan[k], cPWMFreq, cPWMRes);                               // configure PWM channel frequency and resolution
      pinMode(encoder[k].chanA, INPUT);                                        // configure GPIO for encoder channel A input
      pinMode(encoder[k].chanB, INPUT);                                        // configure GPIO for encoder channel B input
      // configure encoder to trigger interrupt with each rising edge on channel A
      attachInterruptArg(encoder[k].chanA, encoderISR, &encoder[k], RISING);
  }

   // Set up SmartLED
   SmartLEDs.begin();                                                          // initialize smart LEDs object (REQUIRED)
   SmartLEDs.clear();                                                          // clear pixel
   SmartLEDs.setPixelColor(0,SmartLEDs.Color(0,0,0));                          // set pixel colors to 'off'
   SmartLEDs.show();                                                           // send the updated pixel colors to the hardware

   pinMode(MOTOR_ENABLE_SWITCH, INPUT_PULLUP);                                 // set up motor enable switch with internal pullup
   pinMode(MODE_BUTTON, INPUT_PULLUP);                                         // Set up mode pushbutton
   modePBDebounce = 0;                                                         // reset debounce timer count
}

void loop()
{
   long pos[] = {0, 0};                                                        // current motor positions
   int pot = 0;                                                                // raw ADC value from pot

   // store encoder positions to avoid conflicts with ISR updates
   noInterrupts();                                                             // disable interrupts temporarily while reading
   for (int k = 0; k < cNumMotors; k++) {
      pos[k] = encoder[k].pos;                                                 // read and store current motor position
   }
   interrupts();                                                               // turn interrupts back on

   currentMicros = micros();                                                   // get current time in microseconds
   if ((currentMicros - previousMicros) >= 1000) {                             // enter when 1 ms has elapsed
      previousMicros = currentMicros;                                          // record current time in microseconds

      // 4 second timer, counts 4000 milliseconds
      timerCount4sec = timerCount4sec + 1;                                     // increment 4 second timer count
      if (timerCount4sec > 4000) {                                             // if 4 seconds have elapsed
         timerCount4sec = 0;                                                   // reset 4 second timer count
         timeUp4sec = true;                                                    // indicate that 4 seconds have elapsed
      }
      
      timerCount3sec = timerCount3sec + 1;                                     // increment 3 second timer count
      if (timerCount3sec > 3000) {                                             // if 3 seconds have elapsed
         timerCount3sec = 0;                                                   // reset 3 second timer count
         timeUp3sec = true;                                                    // indicate that 3 seconds have elapsed
      }
   
      // 2 second timer, counts 2000 milliseconds
      timerCount2sec = timerCount2sec + 1;                                     // increment 2 second timer count
      if (timerCount2sec > 2000) {                                             // if 2 seconds have elapsed
         timerCount2sec = 0;                                                   // reset 2 second timer count
         timeUp2sec = true;                                                    // indicate that 2 seconds have elapsed
      }
   
      // Mode pushbutton debounce and toggle
      if (!digitalRead(MODE_BUTTON)) {                                         // if pushbutton GPIO goes LOW (nominal push)
         // Start debounce
         if (modePBDebounce <= 25) {                                           // 25 millisecond debounce time
            modePBDebounce = modePBDebounce + 1;                               // increment debounce timer count
            if (modePBDebounce > 25) {                                         // if held for at least 25 mS
               modePBDebounce = 1000;                                          // change debounce timer count to 1 second
            }
         }
         if (modePBDebounce >= 1000) {                                         // maintain 1 second timer count until release
            modePBDebounce = 1000;
         }
      }
      else {                                                                   // pushbutton GPIO goes HIGH (nominal release)
         if(modePBDebounce <= 26) {                                            // if release occurs within debounce interval
            modePBDebounce = 0;                                                // reset debounce timer count
         }
         else {
            modePBDebounce = modePBDebounce + 1;                               // increment debounce timer count
            if(modePBDebounce >= 1025) {                                       // if pushbutton was released for 25 mS
               modePBDebounce = 0;                                             // reset debounce timer count
               robotModeIndex++;                                               // switch to next mode
               robotModeIndex = robotModeIndex & 7;                            // keep mode index between 0 and 7
               timerCount3sec = 0;                                             // reset 3 second timer count
               timeUp3sec = false;                                             // reset 3 second timer
            }
         }
      }
  
      // check if drive motors should be powered
      motorsEnabled = !digitalRead(MOTOR_ENABLE_SWITCH);                       // if SW1-1 is on (low signal), then motors are enabled

      // modes 
      // 0 = Default after power up/reset. Robot is stopped.
      // 1 = Press mode button once to enter.        Run robot
         case 0: // Robot stopped
            setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);                          // stop left motor
            setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);                          // stop right motor
            encoder[0].pos = 0;                                                // clear left encoder
            encoder[1].pos = 0;                                                // clear right encoder
            driveIndex = 0;                                                    // reset drive index
            timeUp2sec = false;                                                // reset 2 second timer
            break;

         case 1: // Runs Lab 3 Exercise
            if (timeUp3sec) {                                                  // pause for 3 sec before running case 1 code
               // Read pot to update drive motor speed
               pot = analogRead(POT_R1);
               driveSpeed = map(pot, 0, 4095, cMinPWM, cMaxPWM);
#ifdef DEBUG_DRIVE_SPEED 
               /*Serial.print(F("Drive Speed: Pot R1 = "));
               Serial.print(pot);
               Serial.print(F(", mapped = "));
               Serial.println(driveSpeed);*/
#endif
#ifdef DEBUG_ENCODER_COUNT
               /*Serial.print(F("Left Encoder count = "));
               Serial.print(pos[0]);
               Serial.print(F(" Right Encoder count = "));
               Serial.println(pos[1]);*/
#endif
               if (motorsEnabled) {                                            // run motors only if enabled
                  if (timeUp2sec) {                                            // update drive state after 2 seconds
                     timeUp2sec = false;                                       // reset 2 second timer
                     switch(driveIndex) {                                      // cycle through drive states
                        case 0: // Stops for 2 seconds
                           setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);           // Stops left and right motor
                           setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);
                           driveIndex = 1;                                     // Next state: drive forward
                           break;

                        case 1: // Drives forward 50cm and then back 25cm
                           driveForward(50, driveSpeed);
                           driveBack(25, driveSpeed);
                           driveIndex = 2;                                    // Next state: Pauses before moving backwards again
                           break;

                        case 2: // Pauses for 4 seconds
                        if(timeUp4sec)
                          timeUp4sec = false;   // Resets the 4 second timer
                          driveIndex = 3;       // Next state: Drive backwards another 25cm
                          break;

                        case 3: // Drives backward 25cm
                           driveBack(25, driveSpeed);
                           driveIndex = 4;              // Next state: Pauses for 2 seconds before entering idle mode
                           break;
                        case 4: // Pause 2 seconds
                           robotModeIndex = 0;          // Next robotMode state: Resets to 0, entering idle state
                           break;
                     }
                  }
               }
            }
            else {                                                            // If motors are disabled, they are stopped
               setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);                      
            }
            break;
      }
      // Update brightness of heartbeat display on SmartLED
      displayTime++;                                                          // count milliseconds
      if (displayTime > cDisplayUpdate) {                                     // when display update period has passed
         displayTime = 0;                                                     // reset display counter
         LEDBrightnessIndex++;                                                // shift to next brightness level
         if (LEDBrightnessIndex > sizeof(LEDBrightnessLevels)) {              // if all defined levels have been used
            LEDBrightnessIndex = 0;                                           // reset to starting brightness
         }
         SmartLEDs.setBrightness(LEDBrightnessLevels[LEDBrightnessIndex]);    // set brightness of heartbeat LED
         Indicator();                                                         // update LED
      }
   }
}   

// Set colour of Smart LED depending on robot mode (and update brightness)
void Indicator() {
  SmartLEDs.setPixelColor(0, modeIndicator[robotModeIndex]);                  // set pixel colors to = mode 
  SmartLEDs.show();                                                           // send the updated pixel colors to the hardware
}

// send motor control signals, based on direction and pwm (speed)
void setMotor(int dir, int pwm, int in1, int in2) {
   if (dir == 1) {                                                            // forward
      ledcWrite(in1, pwm);
      ledcWrite(in2, 0);
   }
   else if (dir == -1) {                                                      // reverse
      ledcWrite(in1, 0);
      ledcWrite(in2, pwm);
   }
   else {                                                                     // stop
      ledcWrite(in1, 0);
      ledcWrite(in2, 0);
   }
}

void driveForward(long distanceCm, int driveSpeed){
  // Travel a set distsance via encoder value readings
    long targetCount = distanceCm / distancePerRev;    // Determines required count of wheel revs by dividing the desired distance by the distance travelled per rev
    long initialLeftCount = encoder[0].pos;
    long initialRightCount = encoder[1].pos;

    setMotor(1, driveSpeed, cIN1Chan[0], cIN2Chan[0]);
    setMotor(-1, driveSpeed, cIN1Chan[1], cIN2Chan[1]);

  while (true) {
        // Calculates absolute difference from initial readings
        long traveledLeftCount = abs(encoder[0].pos - initialLeftCount);
        long traveledRightCount = abs(encoder[1].pos - initialRightCount);

        // Checks if robot has traveled the desired distance
        if (traveledLeftCount >= targetCount && traveledRightCount >= targetCount) {
            break; // Breaks out of loop if desired distance is reached
        }
  }
    
    // Stop motors once target distance is reached
    setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);
    setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);
}


void driveBack(long distanceCm, int driveSpeed) {
  // Travel set distance backwards
    long targetCount = distanceCm / distancePerRev;
    long initialLeftCount = encoder[0].pos;
    long initialRightCount = encoder[1].pos;

    // Start driving
    setMotor(-1, driveSpeed, cIN1Chan[0], cIN2Chan[0]);
    setMotor(1, driveSpeed, cIN1Chan[1], cIN2Chan[1]);

    while (true) {
        // Calculates absolute difference from initial readings
        long traveledLeftCount = abs(encoder[0].pos - initialLeftCount);
        long traveledRightCount = abs(encoder[1].pos - initialRightCount);

        // Checks if robot has traveled the desired distance
        if (traveledLeftCount >= targetCount && traveledRightCount >= targetCount) {
            break; // Breaks out of loop if desired distance is reached
        }
    }
    // Stop motors once target distance is reached
    setMotor(0, 0, cIN1Chan[0], cIN2Chan[0]);
    setMotor(0, 0, cIN1Chan[1], cIN2Chan[1]);
}


// encoder interrupt service routine
// argument is pointer to an encoder structure, which is statically cast to a Encoder structure, allowing multiple
// instances of the encoderISR to be created (1 per encoder)
void ARDUINO_ISR_ATTR encoderISR(void* arg) {
   Encoder* s = static_cast<Encoder*>(arg);                                  // cast pointer to static structure
  
   int b = digitalRead(s->chanB);                                            // read state of channel B
   if (b > 0) {                                                              // high, leading channel A
      s->pos++;                                                              // increase position
   }
   else {                                                                    // low, lagging channel A
      s->pos--;                                                              // decrease position
   }
}