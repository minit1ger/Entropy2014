#include "WPILib.h"
#include "IODefinitions.h"
#include "EntropyDrive.h"
#include "EntropyJoystick.h"
#include "ExampleSHS.h"
#include "GenericHID.h"
#include "Math.h"

#define HALFSPEED 1
#define DEADZONE 1
#define DAMPENING 1

const double HALF_SPEED_COEFF = 0.85;
const double DEAD_ZONE_MAX = .15;

class EntropyRobot2014 : public IterativeRobot
{
	// Declare variable for the robot system
		
	// Declare a variable to use to access the driver station object
	DriverStation *EntropyDriverStation;			// driver station object
	UINT32 m_priorPacketNumber;					// keep track of the most recent packet number from the DS
	UINT8 m_dsPacketsReceivedInCurrentSecond;	// keep track of the ds packets received in the current second
	
	// Declare variables for the two joysticks being used
	EntropyJoystick *DriveStick;			// EntropyJoystick used for robot driving
	EntropyJoystick *DriveStick2;		//
	EntropyJoystick *GameStick;			// EntropyJoystick for all other functions		
	float m_turnSpeed;
	
	//Output to Driver Station;
	DriverStationLCD *ds; 
	
	// Declare SHS Subsystems here
	EntropyDrive MyRobot;		// The Robot Drive instance
		
	
	// Local variables to count the number of periodic loops performed
	UINT32 m_autoPeriodicLoops;
	UINT32 m_disabledPeriodicLoops;
	UINT32 m_telePeriodicLoops;

	double previousValue;
	
public:
/**
 * Constructor for this "EntropyRobotDrive2014" Class.
 */
	EntropyRobot2014(void)	{
		printf(" Constructor Started\n");

		// Establish Hardware IO Controllers
		DriveStick = new EntropyJoystick(IODefinitions::USB_PORT_1);
		DriveStick2 = new EntropyJoystick (IODefinitions:: USB_PORT_3);
		GameStick = new EntropyJoystick(IODefinitions::USB_PORT_2);			
	
			
		// Acquire the Driver Station object
		EntropyDriverStation = DriverStation::GetInstance();
		m_priorPacketNumber = 0;
		m_dsPacketsReceivedInCurrentSecond = 0;

		// Initialize counters to record the number of loops completed in autonomous and teleop modes
		m_autoPeriodicLoops = 0;
		m_disabledPeriodicLoops = 0;
		m_telePeriodicLoops = 0;

		ds = DriverStationLCD::GetInstance();
		
		m_turnSpeed=1.0;
		previousValue=0.0;
		printf("EntropyBot14 Constructor Completed\n");
	}
	
	
	/********************************** Init Routines *************************************/

	void RobotInit(void) {
		// Actions which would be performed once (and only once) upon initialization of the
		// robot would be put here.
		
		
		// Initialize SHS Subsystems here
		MyRobot.Initialize();
		
		
		printf("RobotInit() completed.\n");
	}
	
	void DisabledInit(void) {
		m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
		
		// Move the cursor down a few, since we'll move it back up in periodic.
		printf("\x1b[2B");
	}

	void AutonomousInit(void) {
		m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
		
	}

	void TeleopInit(void) {
		m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
		m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
				
		printf("Telop Init completed.\n");
	}

	/********************************** Periodic Routines *************************************/
	
	void DisabledPeriodic(void)  {
		static INT32 printSec = (INT32)GetClock() + 1;
		static const INT32 startSec = (INT32)GetClock();


		// increment the number of disabled periodic loops completed
		m_disabledPeriodicLoops++;
		
		//Disable Drive
		MyRobot.Cleanup();
		//MyShooter.Cleanup();
		
		// while disabled, printout the duration of current disabled mode in seconds
		if (GetClock() > printSec) {
			// Move the cursor back to the previous line and clear it.
			printf("\x1b[1A\x1b[2K");
			printf("Disabled seconds: %d\r\n", printSec - startSec);			
			printSec++;
		}
	}

	void AutonomousPeriodic(void) {
		
		m_autoPeriodicLoops++;
		
		if (m_autoPeriodicLoops <= 35 && m_autoPeriodicLoops >= 0){MyRobot.DriveRobot(0.5,0.0);}
		
		else if (m_autoPeriodicLoops <= 200 && m_autoPeriodicLoops >=90 ) {MyRobot.DriveRobot(-0.2,0.5);}
		
		else if (m_autoPeriodicLoops <= 295 && m_autoPeriodicLoops >=255 ) {MyRobot.DriveRobot(0.3,-0.2);}

		else if (m_autoPeriodicLoops <= 460 && m_autoPeriodicLoops >=350 ) {MyRobot.DriveRobot(-0.2,0.5);}
		
		else {MyRobot.DriveRobot (0,0);}

	}

	
	void TeleopPeriodic(void) {
		// increment the number of teleop periodic loops completed
		m_telePeriodicLoops++;
		
		
		//Using triggers to turn;
		//ds->PrintfLine(DriverStationLCD::kUser_Line1, "GetY: %f",DriveStick->GetY());
		//ds->PrintfLine(DriverStationLCD::kUser_Line2, "GetZ: %f",DriveStick->GetZ());
		//ds->UpdateLCD();
		double YValue=getYValue(DriveStick);
		m_turnSpeed=getHalfSpeed();

		//		
		//      Controller with Z Triggers
		//
				//MyRobot.DriveRobot(YValue,m_turnSpeed*(-DriveStick->GetZ()));
				//
				//Controler for Joystick
				//
				
				//MyRobot.DriveRobot (YValue,m_turnSpeed*(DriveStick->GetX()));
				
				//X Box Controler with left joystick
		//MyRobot.DriveRobot (YValue,(DriveStick->GetRawAxis(Joystick::kDefaultXAxis)));
		
		MyRobot.DriveRobot (YValue,(DriveStick->GetRawAxis(4)));

	} // TeleopPeriodic(void)
	
	double getHalfSpeed(){
		
		double Speed = 1;
		
#ifdef HALFSPEED

		if (DriveStick->GetRawButton (1)){
			
			Speed=HALF_SPEED_COEFF;
			
		}
		
		else {
			
			Speed=1;
		}
#endif
	return Speed;	
	}
	
	double getYValue(EntropyJoystick *js){
		
		double Value=js->GetY();
	 
		double dampValue;
		
#ifdef DAMPENING

if ((Value > 0 and previousValue < 0) or
	(Value < 0 and previousValue > 0)){
	 if ((abs(Value) > .4) and abs(previousValue) > .4){
		 if (abs(Value)+abs(previousValue) >= .1){
			 if (previousValue > Value){
				 Value=previousValue-0.2;
			 }
			 else{
				 Value=previousValue+0.2;
			 }
		 }
	 }
}		
		
#endif 
		
#ifdef DEADZONE
		
		if (Value<DEAD_ZONE_MAX){
			if (Value>-DEAD_ZONE_MAX){
				Value=0;
			}
		}
	
#endif
		previousValue=Value;
	return Value;	
	}


	
	
	
/********************************** Continuous Routines *************************************/

	/* 
	void DisabledContinuous(void) {
	}

	void AutonomousContinuous(void)	{
	}

	void TeleopContinuous(void) {
	}
	*/

	

			
};

START_ROBOT_CLASS(EntropyRobot2014);
