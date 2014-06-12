/******************************************************************************
* File Name          : UF_uArm.cpp
* Author             : Evan
* Updated            : Evan & Scott Gray
* Version            : V0.1 (BATE)
* Date               : 2 May, 2014
* Modified Date      : 29 May 2014
* Description        :
* License            : 
* Copyright(C) 2014 UFactory Team. All right reserved.
*******************************************************************************/

#include "UF_uArm.h"

UF_uArm::UF_uArm()
{
	heightLst  = 0;
	height     = 0;
	stretch    = 0;
	rotation   = 0;
	handRot    = 0;
    lstTime    = 0;
	delay_loop = 0;
    gripperRst  = true;
}

void UF_uArm::init()
{
    // read offset data
    offsetL = EEPROM.read(1);
    offsetR = EEPROM.read(2);
    // initialization the pin
    pinMode(LIMIT_SW, INPUT);  digitalWrite(LIMIT_SW, HIGH);
    pinMode(BTN_D4,   INPUT);  digitalWrite(BTN_D4,   HIGH);
    pinMode(BTN_D7,   INPUT);  digitalWrite(BTN_D7,   HIGH);
    pinMode(BUZZER,   OUTPUT); digitalWrite(BUZZER,   LOW);
    pinMode(PUMP_EN,  OUTPUT); digitalWrite(PUMP_EN,  LOW);
    pinMode(VALVE_EN, OUTPUT); digitalWrite(VALVE_EN, LOW);
	if (EEPROM.read(0) == CALIBRATION_FLAG) // read of offset flag
    {
		// attaches the servo on pin to the servo object
		servoL.attach(SERVO_L, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
		servoR.attach(SERVO_R, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
		servoRot.attach(SERVO_ROT, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
		servoHand.attach(SERVO_HAND, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
		servoHandRot.attach(SERVO_HAND_ROT, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
		servoHand.write(HAND_ANGLE_OPEN);delay(50);
		servoHand.detach();
		// initialization postion
		setPosition(stretch, height, rotation, handRot);
    }
    else
    {	// buzzer alert if calibration needed
		alert(3, 200, 200);
    }
}

void UF_uArm::calibration()
{
    int initPosL = INIT_POS_L + 20; // Added 20 degrees here to start at reasonable point
    int initPosR = INIT_POS_R + 20; // Added 20 degrees here to start at reasonable point

	if(!digitalRead(BTN_D7))
	{
		delay(20);
		// buzzer alert
		alert(1, 20, 0);
	}

	lstTime = millis();
    while(!digitalRead(BTN_D7))
    {
        if(millis() - lstTime > BTN_TIMEOUT_MS)
        {
            // buzzer alert
            alert(2, 50, 100);
            while(!digitalRead(BTN_D7))
            {
                servoL.detach();
                servoR.detach();
            }
			while(1)
			{
				// SG-> Commentary: While user adjusts for minimum angles, keep track of angle and add
				//                  margin of analog reading value of 12, which is about 3 degrees.
				int minAngle_L = readAngle(SERVO_L) + 12;
				int minAngle_R = readAngle(SERVO_R) + 12;

				if(!digitalRead(BTN_D7))
				{
					delay(20);
					// buzzer alert
					alert(1, 20, 0);
					delay(500); //SG-> Added to delay for user to remove hand
					// buzzer alert
					servoL.attach(SERVO_L, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
					servoR.attach(SERVO_R, D150A_SERVO_MIN_PUL, D150A_SERVO_MAX_PUL);
					servoL.write(initPosL);
					servoR.write(initPosR);
					delay(500);
					while(readAngle(SERVO_R) < minAngle_R - SAMPLING_DEADZONE)
					{
						servoR.write(++initPosR);
						delay(50);
					}
					while(readAngle(SERVO_R) > minAngle_R + SAMPLING_DEADZONE)
					{
						servoR.write(--initPosR);
						delay(50);
					}
					while(readAngle(SERVO_L) < minAngle_L - SAMPLING_DEADZONE)
					{
						servoL.write(++initPosL);
						delay(50);
					}
					while(readAngle(SERVO_L) > minAngle_L + SAMPLING_DEADZONE)
					{
						servoL.write(--initPosL);
						delay(50);
					}
					offsetL = initPosL - INIT_POS_L;
					offsetR = initPosR - INIT_POS_R;
					EEPROM.write(0, CALIBRATION_FLAG);  // Added flag to know if offset is done
					EEPROM.write(1, offsetL);			// offsetL
					EEPROM.write(2, offsetR);			// offsetR
					// buzzer alert
					alert(1, 500, 0);      
					// reset postion
					init();
					break;
				}
			}
        }
    }
}

void UF_uArm::setPosition(double _stretch, double _height, int _armRot, int _handRot)
{
	_armRot = -_armRot;
	if(!digitalRead(LIMIT_SW) && _height < heightLst) //limit switch protection
	_height = heightLst;
	// input limit
	_stretch = constrain(_stretch, ARM_STRETCH_MIN,   ARM_STRETCH_MAX) + 55;		// +55, set stretch zero
	_height  = constrain(_height,  ARM_HEIGHT_MIN,    ARM_HEIGHT_MAX);
	_armRot  = constrain(_armRot,  ARM_ROTATION_MIN,  ARM_ROTATION_MAX) + 90;		// +90, change -90~90 to 0~180
	_handRot = constrain(_handRot, HAND_ROTATION_MIN, HAND_ROTATION_MAX) + 90;	// +90, change -90~90 to 0~180
	// angle calculation
	double stretch2height2 = _stretch * _stretch + _height * _height;              //
	double angleA = (acos( (ARM_A2B2 - stretch2height2) / ARM_2AB )) * RAD_TO_DEG; // angle between the upper and the lower
	double angleB = (atan(_height/_stretch)) * RAD_TO_DEG;                         // 
	double angleC = (acos((ARM_A2 + stretch2height2 -ARM_B2)/(2 * ARM_A * sqrt(stretch2height2)))) * RAD_TO_DEG; // 
	int angleR = 180 - angleA - angleB - angleC + FIXED_OFFSET_R + offsetR;        // 
	int angleL = angleB + angleC + FIXED_OFFSET_L + offsetL;                       // 
	// angle limit
	angleL = constrain(angleL, 10 + offsetL, 145 + offsetL);
	angleR = constrain(angleR, 25 + offsetR, 150 + offsetR);
	angleR = constrain(angleR, angleL - 90 + offsetR, angleR);	// behind  -120+30 = -90
	if(angleL<15+offsetL)
	angleR = constrain(angleR, 70 + offsetR, angleR);			// front down
	// set servo position
	servoR.write(angleR);
	servoL.write(angleL);
	servoRot.write(_armRot);
	servoHandRot.write(_handRot);
	heightLst = _height;
}

int UF_uArm::readAngle(char _servoNum)
{
	int portAd;
	switch(_servoNum)
	{
		case SERVO_L:
			portAd = A0;
			break;
		case SERVO_R:
			portAd = A1;
			break;
		case SERVO_ROT:
			portAd = A2;
			break;
		case SERVO_HAND_ROT:
			portAd = A3;
			break;
		case SERVO_HAND:
			portAd = A4;
			break;
		default: return 0; break;
	}
	int adAdd = 0;
	for(char i=0; i<5; i++)
	{
		adAdd += analogRead(portAd);
	}
	return adAdd/5;
}

void UF_uArm::gripperCatch()
{
	servoHand.attach(SERVO_HAND, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
    servoHand.write(HAND_ANGLE_CLOSE);
    digitalWrite(VALVE_EN, LOW); // valve disnable
    digitalWrite(PUMP_EN, HIGH); // pump enable
    gripperRst = true;
}

void UF_uArm::gripperRelease()
{
	if(gripperRst)
    {
      servoHand.attach(SERVO_HAND, D009A_SERVO_MIN_PUL, D009A_SERVO_MAX_PUL);
      servoHand.write(HAND_ANGLE_OPEN);
      digitalWrite(VALVE_EN, HIGH); // valve enable, decompression
      digitalWrite(PUMP_EN, LOW);   // pump disnable
      gripperRst = false;
      delay_loop = 0;
    }
}

void UF_uArm::gripperDetach()
{
    if(++delay_loop > 300000)        // delay release valve
    {
        servoHand.detach();
        digitalWrite(VALVE_EN, LOW); // valve disnable
        delay_loop=0;
    }
}

void UF_uArm::gripperDirectDetach()
{
    servoHand.detach();
    digitalWrite(VALVE_EN, LOW); // valve disnable
}

void UF_uArm::pumpOn()
{
    digitalWrite(PUMP_EN, HIGH);    // pump enable
}

void UF_uArm::pumpOff()
{
    digitalWrite(PUMP_EN, LOW);     // pump disnable
}

void UF_uArm::valveOn()
{
    digitalWrite(VALVE_EN, HIGH);   // valve enable, decompression
}

void UF_uArm::valveOff()
{
    digitalWrite(VALVE_EN, LOW);    // valve disnable
}

void UF_uArm::detachServo(char _servoNum)
{
    switch(_servoNum)
    {
        case SERVO_L:
        servoL.detach();
        break;
        case SERVO_R:
        servoR.detach();
        break;
        case SERVO_ROT:
        servoRot.detach();
        break;
        case SERVO_HAND_ROT:
        servoHandRot.detach();
        break;
        case SERVO_HAND:
        servoHand.detach();
        break;
        default: break;
    }
}

void UF_uArm::sendData(byte _dataAdd, int _dataIn)
{
	Serial.write(0xFF); Serial.write(0xAA); // send data head
	Serial.write(_dataAdd);
	Serial.write(*((char *)(&_dataIn) + 1));
	Serial.write(*((char *)(&_dataIn)));
}

void UF_uArm::alert(int _times, int _runTime, int _stopTime)
{
	for(int _ct=0; _ct < _times; _ct++)
	{
		delay(_stopTime);
		digitalWrite(BUZZER, HIGH);
		delay(_runTime);
		digitalWrite(BUZZER, LOW);
	}
}