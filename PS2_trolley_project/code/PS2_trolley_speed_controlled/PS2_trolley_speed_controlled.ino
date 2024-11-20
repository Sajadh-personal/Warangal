/*******************************************************************************
 *                                                                             *
 *    Speed-controlled-PS2-Trolley based on library example                    *
 *                'PsxControllerShieldDemo'                                    *
 *                                                                             *
********************************************************************************/

// PsxControllerShield connects controller to HW SPI port through ICSP connector
#include <PsxControllerHwSpi.h>
#include <DigitalIO.h>

#include <avr/pgmspace.h>
typedef const __FlashStringHelper * FlashStr;
typedef const byte* PGM_BYTES_P;
#define PSTR_TO_F(s) reinterpret_cast<const __FlashStringHelper *> (s)
#define    ON                 HIGH
#define    OFF                LOW
#define    FORWARD            HIGH
#define    REVERSE            LOW
#define    R_LOGIC_ON         LOW
#define    R_LOGIC_OFF        HIGH
#define    R_LOGIC_FORWARD    LOW
#define    R_LOGIC_REVERSE    HIGH
#define    MOTOR1_DIRECTION   7  //2
#define    MOTOR1_CONTROL     6  //3
#define    MOTOR2_DIRECTION   4
#define    MOTOR2_CONTROL     5
#define    MOTOR3_CONTROL     3  // 6
#define    MOTOR3_DIRECTION   2  // 7
#define    MOTOR3_DIRECTION2   9  //2


#define    mask_L2            256
#define    mask_L1            1024

char reverse_bit; 

const byte PIN_PS2_ATT = 10;
const byte PIN_HAVECONTROLLER = 8;
//const byte PIN_BUTTONPRESS = 9; //7;
//const byte PIN_ANALOG = 6;

const byte ANALOG_DEAD_ZONE = 50U;

PsxControllerHwSpi<PIN_PS2_ATT> psx;

boolean haveController = false;

const char buttonSelectName[] PROGMEM = "Select";
const char buttonL3Name[] PROGMEM = "L3";
const char buttonR3Name[] PROGMEM = "R3";
const char buttonStartName[] PROGMEM = "Start";
const char buttonUpName[] PROGMEM = "Up";
const char buttonRightName[] PROGMEM = "Right";
const char buttonDownName[] PROGMEM = "Down";
const char buttonLeftName[] PROGMEM = "Left";
const char buttonL2Name[] PROGMEM = "L2";
const char buttonR2Name[] PROGMEM = "R2";
const char buttonL1Name[] PROGMEM = "L1";
const char buttonR1Name[] PROGMEM = "R1";
const char buttonTriangleName[] PROGMEM = "Triangle";
const char buttonCircleName[] PROGMEM = "Circle";
const char buttonCrossName[] PROGMEM = "Cross";
const char buttonSquareName[] PROGMEM = "Square";

const char* const psxButtonNames[PSX_BUTTONS_NO] PROGMEM = {
	buttonSelectName,
	buttonL3Name,
	buttonR3Name,
	buttonStartName,
	buttonUpName,
	buttonRightName,
	buttonDownName,
	buttonLeftName,
	buttonL2Name,
	buttonR2Name,
	buttonL1Name,
	buttonR1Name,
	buttonTriangleName,
	buttonCircleName,
	buttonCrossName,
	buttonSquareName
};

byte psxButtonToIndex (PsxButtons psxButtons) 
{
	byte i;
	for (i = 0; i < PSX_BUTTONS_NO; ++i) 
  {
		if (psxButtons & 0x01) 
    {
			break;
		}
		psxButtons >>= 1U;
	}
	return i;
}

FlashStr getButtonName (PsxButtons psxButton) 
{
	FlashStr ret = F("");
	
	byte b = psxButtonToIndex (psxButton);
	if (b < PSX_BUTTONS_NO) 
  {
		PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(psxButtonNames[b])));
		ret = PSTR_TO_F (bName);
	}

	return ret;
}

void dumpButtons (PsxButtons psxButtons) 
{
	static PsxButtons lastB = 0;
	if (psxButtons != lastB) {
		lastB = psxButtons;     // Save it before we alter it
		
		Serial.print (F("Pressed: "));

		for (byte i = 0; i < PSX_BUTTONS_NO; ++i) 
    {
			byte b = psxButtonToIndex (psxButtons);
			if (b < PSX_BUTTONS_NO) 
      {
				PGM_BYTES_P bName = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(psxButtonNames[b])));
				Serial.print (PSTR_TO_F (bName));
			}
			psxButtons &= ~(1 << b);
			if (psxButtons != 0) 
      {
				Serial.print (F(", "));
			}
		}
		Serial.println ();
	}
}

void dumpAnalog (const char *str, const int8_t x, const int8_t y) 
{
	Serial.print (str);
	Serial.print (F(" analog: x = "));
	Serial.print (x);
	Serial.print (F(", y = "));
	Serial.println (y);
}

// We like analog sticks to return something in the [-127, +127] range
boolean rightAnalogMoved (int8_t& x, int8_t& y) 
{
	boolean ret = false;
	byte rx, ry;
	
	if (psx.getRightAnalog (rx, ry)) 				// [0 ... 255]
  {
		int8_t deltaRX = rx - ANALOG_IDLE_VALUE;	// [-128 ... 127]
		if (abs (deltaRX) > ANALOG_DEAD_ZONE) 
    {
			x = deltaRX;
			if (x == -128)
				x = -127;
			ret = true;
		} 
    else 
    {
			x = 0;
		}
		
		int8_t deltaRY = ry - ANALOG_IDLE_VALUE;
		if (abs (deltaRY) > ANALOG_DEAD_ZONE) 
    {
			y = deltaRY;
			if (y == -128)
				y = -127;
			ret = true;
		} 
    else 
    {
			y = 0;
		}
	}

	return ret;
}

boolean leftAnalogMoved (int8_t& x, int8_t& y) 
{
	boolean ret = false;
	byte lx, ly;
	
	if (psx.getLeftAnalog (lx, ly)) 				// [0 ... 255]
  {
		if (psx.getProtocol () != PSPROTO_NEGCON && psx.getProtocol () != PSPROTO_JOGCON) 
    {
			int8_t deltaLX = lx - ANALOG_IDLE_VALUE;	// [-128 ... 127]
			uint8_t deltaLXabs = abs (deltaLX);
			if (deltaLXabs > ANALOG_DEAD_ZONE) 
      {
				x = deltaLX;
				if (x == -128)
					x = -127;
				ret = true;
			} 
      else 
      {
				x = 0;
			}
			
			int8_t deltaLY = ly - ANALOG_IDLE_VALUE;
			uint8_t deltaLYabs = abs (deltaLY);
			if (deltaLYabs > ANALOG_DEAD_ZONE) 
      {
				y = deltaLY;
				if (y == -128)
					y = -127;
				ret = true;
			} 
      else 
      {
				y = 0;
			}
		} 
    else 
    {
			// The neGcon and JogCon are more precise and work better without any dead zone
			x = lx, y = ly;
		}
	}
	return ret;
}


// Controller Type
const char ctrlTypeUnknown[] PROGMEM = "Unknown";
const char ctrlTypeDualShock[] PROGMEM = "Dual Shock";
const char ctrlTypeDsWireless[] PROGMEM = "Dual Shock Wireless";
const char ctrlTypeGuitHero[] PROGMEM = "Guitar Hero";
const char ctrlTypeOutOfBounds[] PROGMEM = "(Out of bounds)";

const char* const controllerTypeStrings[PSCTRL_MAX + 1] PROGMEM = {
	ctrlTypeUnknown,
	ctrlTypeDualShock,
	ctrlTypeDsWireless,
	ctrlTypeGuitHero,
	ctrlTypeOutOfBounds
};


// Controller Protocol
const char ctrlProtoUnknown[] PROGMEM = "Unknown";
const char ctrlProtoDigital[] PROGMEM = "Digital";
const char ctrlProtoDualShock[] PROGMEM = "Dual Shock";
const char ctrlProtoDualShock2[] PROGMEM = "Dual Shock 2";
const char ctrlProtoFlightstick[] PROGMEM = "Flightstick";
const char ctrlProtoNegcon[] PROGMEM = "neGcon";
const char ctrlProtoJogcon[] PROGMEM = "Jogcon";
const char ctrlProtoOutOfBounds[] PROGMEM = "(Out of bounds)";

const char* const controllerProtoStrings[PSPROTO_MAX + 1] PROGMEM = {
	ctrlProtoUnknown,
	ctrlProtoDigital,
	ctrlProtoDualShock,
	ctrlProtoDualShock2,
	ctrlProtoFlightstick,
	ctrlProtoNegcon,
	ctrlProtoJogcon,
	ctrlTypeOutOfBounds
};

void motor1_fwd()
{
    digitalWrite( MOTOR1_DIRECTION, R_LOGIC_FORWARD);
    digitalWrite( MOTOR1_CONTROL, R_LOGIC_ON);
}
void motor1_reverse()
{
    digitalWrite( MOTOR1_DIRECTION, R_LOGIC_REVERSE);
    digitalWrite( MOTOR1_CONTROL, R_LOGIC_ON);
}
void motor1_stop()
{
    digitalWrite( MOTOR1_DIRECTION, R_LOGIC_REVERSE);
    digitalWrite( MOTOR1_CONTROL, R_LOGIC_OFF);
}
void motor2_fwd()
{
    digitalWrite( MOTOR2_DIRECTION, R_LOGIC_FORWARD);
    digitalWrite( MOTOR2_CONTROL, R_LOGIC_ON);
}
void motor2_reverse()
{
    digitalWrite( MOTOR2_DIRECTION, R_LOGIC_REVERSE);
    digitalWrite( MOTOR2_CONTROL, R_LOGIC_ON);
}
void motor2_stop()
{
    digitalWrite(MOTOR2_DIRECTION, R_LOGIC_REVERSE);
    digitalWrite(MOTOR2_CONTROL, R_LOGIC_OFF);
}
void motor3_fwd()
{
    digitalWrite( MOTOR3_DIRECTION, FORWARD);
    digitalWrite( MOTOR3_DIRECTION2, REVERSE);
}
void motor3_reverse()
{
    digitalWrite( MOTOR3_DIRECTION, REVERSE);
    digitalWrite( MOTOR3_DIRECTION2, FORWARD);
}
void motor3_stop()
{
    digitalWrite(MOTOR3_DIRECTION, REVERSE);
    digitalWrite( MOTOR3_DIRECTION2, REVERSE);
    digitalWrite(MOTOR3_CONTROL, OFF);
}

void trolley_control(PsxButtons button)
{
#if 0
    Serial.println("==============");
    Serial.print("button : ");
    Serial.print(button);
    Serial.print("  mask : ");
    Serial.println(mask_L1 & button);
  	Serial.println("==============");
#endif

    if(button & mask_L2)
    {
        motor1_fwd();
    }
    else if(button & mask_L1)
    {
        motor1_reverse();
    }
    else
    {
        motor1_stop();
    }
}
void move_with_joystick(int8_t x, int8_t y)
{
    int    speed;  
    if (y < -50)
    {
        reverse_bit=0;
        speed = map(y, -50, -127, 0, 255);
        analogWrite(MOTOR3_CONTROL, speed);
        motor3_fwd();
    }
    else if(y > 50)
    {
        if(reverse_bit < 2)
        {
            reverse_bit=1;
            return ;
        }
        
        motor3_reverse();
        speed = map(y, 50, 127, 0, 255);
        analogWrite(MOTOR3_CONTROL, speed);
        Serial.print("reverse...");
    }
    else
    {
        motor3_stop();
        if(reverse_bit == 1)
        {
            reverse_bit=2;
        }
    }
    
    if (x < -50)
    {
        motor2_fwd();
    }
    else if(x > 50)
    {
        motor2_reverse();
    }
    else
    {
        motor2_stop();
    }
}

void setup () 
{
  pinMode(MOTOR1_DIRECTION, OUTPUT);
  pinMode(MOTOR1_CONTROL, OUTPUT);
	pinMode(MOTOR2_DIRECTION, OUTPUT);
  pinMode(MOTOR2_CONTROL, OUTPUT);
	pinMode(MOTOR3_DIRECTION, OUTPUT);
  pinMode(MOTOR3_CONTROL, OUTPUT);
  pinMode(MOTOR3_DIRECTION2,OUTPUT);
	motor1_stop();
  motor2_stop();
  motor3_stop();
  
  fastPinMode (PIN_HAVECONTROLLER, OUTPUT);
	//fastPinMode (PIN_BUTTONPRESS, OUTPUT);
	//fastPinMode (PIN_ANALOG, OUTPUT);
	Serial.begin (115200);
  Serial.println ("starting...");
  //while(1);
#if 1
  //relay beep
  analogWrite(MOTOR1_CONTROL, 140);
  delay(50);
  digitalWrite(MOTOR1_CONTROL, HIGH);
  delay(100);
  analogWrite(MOTOR1_CONTROL, 140);
  delay(50);
  digitalWrite(MOTOR1_CONTROL, HIGH);
#endif    
	delay (300);
  
	
	while (!Serial) 
  {
		fastDigitalWrite (PIN_HAVECONTROLLER, (millis () / 333) % 2);
		//fastDigitalWrite (PIN_BUTTONPRESS, (millis () / 333) % 2);
	}
	Serial.println (F("Ready!"));
}
 
void loop () 
{
	static int8_t slx, sly, srx, sry;
	fastDigitalWrite (PIN_HAVECONTROLLER, haveController);
	
	if (!haveController) 
  {
		if (psx.begin ()) 
    {
			Serial.println (F("Controller found!"));
			delay (300);
			if (!psx.enterConfigMode ()) 
      {
				Serial.println (F("Cannot enter config mode"));
			} 
      else 
      {
				PsxControllerType ctype = psx.getControllerType ();
				PGM_BYTES_P cname = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(controllerTypeStrings[ctype < PSCTRL_MAX ? static_cast<byte> (ctype) : PSCTRL_MAX])));
				Serial.print (F("Controller Type is: "));
				Serial.println (PSTR_TO_F (cname));

				if (!psx.enableAnalogSticks ()) 
        {
					Serial.println (F("Cannot enable analog sticks"));
				}
				
				if (!psx.enableAnalogButtons ()) 
        {
					Serial.println (F("Cannot enable analog buttons"));
				}
				
				if (!psx.exitConfigMode ()) 
        {
					Serial.println (F("Cannot exit config mode"));
				}
			}

			psx.read ();		// Make sure the protocol is up to date
			PsxControllerProtocol proto = psx.getProtocol ();
			PGM_BYTES_P pname = reinterpret_cast<PGM_BYTES_P> (pgm_read_ptr (&(controllerProtoStrings[proto < PSPROTO_MAX ? static_cast<byte> (proto) : PSPROTO_MAX])));
			Serial.print (F("Controller Protocol is: "));
			Serial.println (PSTR_TO_F (pname));

			haveController = true;
		}
	} 
  else 
  {
		if (!psx.read ()) 
    {
			Serial.println (F("Controller lost :("));
			haveController = false;
		} 
    else 
    {
			fastDigitalWrite (NULL, !!psx.getButtonWord ());
			dumpButtons (psx.getButtonWord ());
      trolley_control(psx.getButtonWord());
			int8_t lx = 0, ly = 0;
			leftAnalogMoved (lx, ly);
			if (lx != slx || ly != sly) 
      {
				dumpAnalog ("Left", lx, ly);
				slx = lx;
				sly = ly;
			}

			int8_t rx = 0, ry = 0;
			rightAnalogMoved (rx, ry);
			if (rx != srx || ry != sry) 
      {
				dumpAnalog ("Right", rx, ry);
        move_with_joystick(rx, ry);
				srx = rx;
				sry = ry;
			}
		}
	}
	delay (1000 / 60);
}