#include "WPILib.h"
#include<thread>
#include <Math.h>

using namespace std;

#define DEBUG 								//Debug mode, shows stats as well as other functions for testing
//#define THROTTLE							//Sets rollers to be controlled by the a_stick throttle

#define TRACKMULT .9 						//Multiplier for vision tracking speed

#define ARM_BOTTOM_POSITION -6.0 			//Arm encoder positions
#define ARM_UP_POSITION -590
#define ARM_CLIMB_POSITION -656
#define ARM_TRAVERSE_POSITION -190


#define WINCH_PORT_POSITION 4721
#define WINCH_ZERO_POSITION 0

#define SHIFT_UP_SPEED 3000 				//Encoder speed(rate) limits for shifting gears
#define SHIFT_DOWN_SPEED 2500

#define R_TRAIN(x)r_motor1.x;r_motor2.x; 	//sets all the motors in each drive train or roller
#define L_TRAIN(x)l_motor1.x;l_motor2.x;
//#define ROLLERS(x)roller7.x;roller8.x;


class Robot: public SampleRobot
{
	Joystick l_stick;
	Joystick r_stick;
	Joystick a_stick;

	CANTalon l_motor1;
	CANTalon l_motor2;
	CANTalon r_motor1;
	CANTalon r_motor2;
	CANTalon winch5;
	CANTalon a_motor6;
	CANTalon roller7;
	CANTalon roller8;
	CANTalon a_motor9;

	Compressor cmprssr;

	std::shared_ptr<NetworkTable> table; 	//holds GRIP vision tracking values
	std::vector<double> centerX; 			//values from the NetworkTable
	std::vector<double> centerY;
	std::vector<double> area;

	Encoder armEncoderLeft;
	Encoder r_drive,l_drive; 				//Left and right motor encoders

	Solenoid shooter,shift;

	float armPosition; 						//Keeps track of the arm position
	float armSetPoint;
	float winchPosition;
	float winchSetPoint;
	bool b_shift=1; 						//Whether or not in high gear (Solenoid on/off)
	double avg_speed;

	typedef enum {
		Floor,
		Shoot,
		Climb,
		Traverse
	}ArmPosition;

	typedef enum {
		Port,
		Zero
	}WinchPosition;




public:
	Robot() :
			l_stick(0),
			r_stick(1),
			a_stick(2),
			l_motor1(1),
			l_motor2(2),
			r_motor1(3),
			r_motor2(4),
			winch5(5),
			a_motor6(6),
			roller7(7),
			roller8(8),
			a_motor9(9),
			armEncoderLeft(0,1,Encoder::k4X), 	//Encoder A channel on port 0, Encoder B channel on port 1
			r_drive(2,3,Encoder::k4X),
			l_drive(4,5,Encoder::k4X),
			shooter(1),
			shift(0)

	{
		armPosition = 0.0;
		armSetPoint = 0.0; 											//Start the setpoint at 0
		winchPosition = 0.0;
		winchSetPoint = 0.0;
		a_motor6.ConfigNeutralMode(CANTalon::kNeutralMode_Brake); 	//Sets the arm motors to break mode
		a_motor9.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
		winch5.SetEncPosition(0);
		roller7.SetVoltageRampRate(24);
		roller8.SetVoltageRampRate(24);

		winch5.ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
		/*l_motor2.ConfigNeutralMode(CANTalon::kNeutralMode_Coast);
		r_motor2.ConfigNeutralMode(CANTalon::kNeutralMode_Coast); */
	}


	void Autonomous()
	{
		StopMotors();

		/*std::string autoSelected = SmartDashboard::GetString("Auto Selector", autoNameDefault);

		if(autoSelected == autoNameCustom){
			//Custom Auto goes here

		} else {
			//Default Auto goes here

		}*/
	}


	void OperatorControl()
	{
#ifdef DEBUG
		PowerDistributionPanel pdp; 				//Initializes the pdp if DEBUG is not commented out at the top
#endif // DEBUG

		StopMotors(); 								//Stops all motors before beginning to drive
		SmartDashboard::PutString("DB/String 3", std::to_string(armEncoderLeft.GetDistance()));
		while (IsOperatorControl() && IsEnabled())
		{
			/*
#ifdef DEBUG
#ifdef THROTTLE
			ROLLERS(Set(a_stick.GetThrottle()));	//If both DEBUG and THROTTLE are defined,
													//then set the roller speed to the throttle
#else
			if(a_stick.GetRawButton(7))				//If DEBUG, but not THROTTLE,
				ROLLERS(Set(-1));					//then set rollers when button are pressed
			if(a_stick.GetRawButton(8))
				ROLLERS(Set(1));
			if(!(a_stick.GetRawButton(8)||a_stick.GetRawButton(7)))
				ROLLERS(Set(0));
#endif // THROTTLE
*/
													//If DEBUG, then display PDP and Talon values in SmartDahsboard Strings
			SmartDashboard::PutString("DB/String 5",
					(string)"PDP 1: "  + std::to_string(pdp.GetCurrent(1)) + (string)"A");
			SmartDashboard::PutString("DB/String 6",
					(string)"PDP 14: " + std::to_string(pdp.GetCurrent(14)) + (string)"A");

			SmartDashboard::PutString("DB/String 8",
					(string)"Talon 7: " + std::to_string(roller7.GetOutputCurrent()) + (string)"A");
			SmartDashboard::PutString("DB/String 9",
					(string)"Talon 8: " + std::to_string(roller8.GetOutputCurrent()) + (string)"A");

			SmartDashboard::PutString("DB/String 1", std::to_string(l_drive.GetRate()).c_str());
			SmartDashboard::PutString("DB/String 2", std::to_string(-r_drive.GetRate()).c_str());

			SmartDashboard::PutString("DB/String 3", (string)"ArmEncoder: " + std::to_string(armEncoderLeft.GetDistance()));
			SmartDashboard::PutString("DB/String 7", (string)"WincEncoder: " + std::to_string(winch5.GetEncPosition()));


			//shooter.Set(a_stick.GetRawButton(1)); //If DEBUG, then shoot on a_stick trigger press
			if(a_stick.GetRawButton(1))
			{
				shooter.Set(true);
			}
			else
			{
				shooter.Set(false);
			}

//#endif // DEBUG

			table = NetworkTable::GetTable("GRIP/myContoursReport");
		    centerX = table->GetNumberArray("centerX", llvm::ArrayRef<double>());
		    centerY = table->GetNumberArray("centerY", llvm::ArrayRef<double>());
		    area = table->GetNumberArray("area", llvm::ArrayRef<double>());

		    if(a_stick.GetRawButton(2))
		    {
		    	DriveWinchToPosition(Port);
		    }
		    else if(a_stick.GetRawButton(3))
		    {
		    	DriveWinchToPosition(Zero);
		    }
		    else
		    {
		    winch5.Set(a_stick.GetY());
		    }


			if(l_stick.GetRawButton(1))
				TrackTarget();

			else if (r_stick.GetRawButton(1))
				TrackTargetPivot();

		/*	else if (l_stick.GetRawButton(2))
			{
				FollowMe();
			} */

			else
				TankDrive();

			/*if(l_stick.GetRawButton(2))
			{
				DriveArmToPosition(Floor); //Set to the down position
			}
			else if(l_stick.GetRawButton(3))
			{
				DriveArmToPosition(Shoot); //Set to the up position
			}*/
/*
#ifdef DEBUG
			a_motor6.Set(a_stick.GetY());		//If DEBUG, then control arm with a_stick
			a_motor9.Set(-a_stick.GetY());
#endif // DEBUG
*/

			//Roller control
			if(a_stick.GetRawButton(7))
			{
				roller7.Set(1);
				roller8.Set(1);
			}
			else if(a_stick.GetRawButton(8))
			{
				roller7.Set(-1);
				roller8.Set(-1);
			}
			else
			{
				roller7.Set(0);
				roller8.Set(0);
			}

			//Arm position control

			/*a_motor6.Set(a_stick.GetY());
			a_motor9.Set(-a_stick.GetY());
			*/

			if(a_stick.GetRawButton(11))
			{
				//armSetPoint = ARM_BOTTOM_POSITION;
				DriveArmToPosition(Floor);
			}
			else if(a_stick.GetRawButton(12))
			{
				DriveArmToPosition(Shoot);
			}
			else if(a_stick.GetRawButton(9))
			{
				DriveArmToPosition(Climb);
			}
			else
			{
				DriveArmToPosition(Traverse);
			}


			if(l_stick.GetRawButton(6))
				shift.Set(0);

			else
				AutoShift();

			Wait(0.005);						// wait for a motor update time
		}
	}


	void AutoShift() //Automatically shifts gears based on average speed of motors
	{
		avg_speed = (fabs(l_drive.GetRate())+fabs(r_drive.GetRate()))/2;
		if(avg_speed>SHIFT_UP_SPEED)
			shift.Set(1);
		else if(avg_speed<SHIFT_DOWN_SPEED)
			shift.Set(0);
	}


	void StopMotors() //Sets all motors to 0
	{
		l_motor1.Set(0);
		l_motor2.Set(0);
		r_motor1.Set(0);
		r_motor2.Set(0);
		winch5.Set(0);
		a_motor6.Set(0);
		roller7.Set(0);
		roller8.Set(0);
		a_motor9.Set(0);
	}


	void DriveArmToPosition(ArmPosition setpoint) //Sets arm in position 'setpoint'
	{

		//First give the encoder a setpoint to drive to
		switch(setpoint){
		case Floor:
			armSetPoint = ARM_BOTTOM_POSITION;
			break;
		case Shoot:
			armSetPoint = ARM_UP_POSITION;
			break;
		case Climb:
			armSetPoint = ARM_CLIMB_POSITION;
			break;
		case Traverse:
			armSetPoint = ARM_TRAVERSE_POSITION;
			break;
		default:
			std::cerr<<"Non-existant arm position setting"<<endl;
		}

		armPosition = armEncoderLeft.GetDistance();
		float error = armPosition - armSetPoint;
		float motorOutputBeforeSqrt = (error/650);
		float motorOutput;
		if(motorOutputBeforeSqrt < 0) //Make sure we don't take the sqrt of a negative number
		{
			motorOutputBeforeSqrt = -motorOutputBeforeSqrt;
			motorOutput = cbrt(motorOutputBeforeSqrt);
			motorOutput = -motorOutput;
		}
		else
		{
			motorOutput = cbrt(motorOutputBeforeSqrt);
		}
		if(motorOutput < 0)
		{
		motorOutput = motorOutput/2;
		}

		if( (armSetPoint == ARM_BOTTOM_POSITION ) && (error < 20 && error > -20) )
		{
			a_motor9.Set(0.0);
			a_motor6.Set(0.0);
		}
		else
		{
		a_motor9.Set(motorOutput);
		a_motor6.Set(-motorOutput);
		}
		SmartDashboard::PutString("DB/String 4", (string)"MotorOutput " + std::to_string(motorOutput));
		/*
		if( (armPosition - armSetPoint) / 30 < .3) //Test these numbers
		{
			a_motor9.Set(-.3); //and these
			a_motor6.Set(.3);
		}
		else if( (armPosition - armSetPoint) / 30  > -.65) //and these
		{
			a_motor9.Set(-.65); //and these
			a_motor6.Set(.65);
		}
		else
		{
			a_motor9.Set(- (armPosition - armSetPoint) / 30); //and these
			a_motor6.Set((armPosition - armSetPoint) / 30);
		}
*/
	}

	void DriveWinchToPosition(WinchPosition setpoint) //Sets arm in position 'setpoint'
		{

			//First give the encoder a setpoint to drive to
			switch(setpoint){
			case Port:
				winchSetPoint = WINCH_PORT_POSITION;
				break;
			case Zero:
				winchSetPoint = WINCH_ZERO_POSITION;
				break;

			default:
				std::cerr<<"Non-existant arm position setting"<<endl;
			}

			winchPosition = winch5.GetEncPosition();
			float error = winchPosition - winchSetPoint;
			float motorOutputBeforeSqrt = error / 1500;
			//float motorOutput;

			winch5.Set(motorOutputBeforeSqrt);


			//SmartDashboard::PutString("DB/String 4", (string)"MotorOutput " + std::to_string(motorOutput));

		}



	void GearShift() //Shifts gears based on a single button press
	{
		if(l_stick.GetRawButton(6))
		{
			shift.Set(0);
			b_shift=0;
		}
		else if(!b_shift)
		{

			shift.Set(1);
			b_shift=1;
		}
	}


	void TankDrive() //Drives each train based on each joystick
	{
		R_TRAIN(Set(-r_stick.GetY()));
		L_TRAIN(Set(l_stick.GetY()));
	}


	void TrackTarget() //Turns towards any target in sight with only the right train
	{
		double distoff = 0.0;

		if(!centerX.empty())
		{
			distoff = 160 - centerX[0];

				if(centerX[0] < 140.00)
				{
					//l_motor1.Set(distoff/160 * TRACKMULT);
					r_motor1.Set(distoff/160 * TRACKMULT);
				}

				else if(centerX[0] > 180.00)
				{
					//l_motor1.Set(distoff/160 * TRACKMULT);
					r_motor1.Set(distoff/160 * TRACKMULT);
				}

				else if(centerX[0] > 140 && centerX[0] < 180)
				{
					l_motor1.Set(0);
					r_motor1.Set(0);
				}
		}
	}


	void TrackTargetPivot() //Turns towards any target in sight with both trains
	{
		double distoff = 0.0;

		if(!centerX.empty())
		{
			distoff = 160 - centerX[0];
				if(centerX[0] < 140.00)
				{
					l_motor1.Set(distoff/160 * TRACKMULT);
					r_motor1.Set(distoff/160 * TRACKMULT);
				}

				else if(centerX[0] > 180.00)
				{
					l_motor1.Set(distoff/160 * TRACKMULT);
					r_motor1.Set(distoff/160 * TRACKMULT);
				}

				else if(centerX[0] > 140 && centerX[0] < 180)
				{
					l_motor1.Set(0);
					r_motor1.Set(0);
				}
		}
	}


	void FollowMe() //Follows target in a straight line by measuring area of target
	{
		if (area[0] > 1000)
		{
			L_TRAIN(Set(0));
			R_TRAIN(Set(0));
		}

		else if (area[0] < 500)
		{
			L_TRAIN(Set(0.3));
			R_TRAIN(Set(-0.3));
		}

		else if (area[0] < 1000 && area[0] > 500)
		{
			L_TRAIN(Set(0.2));
			R_TRAIN(Set(-0.2));
		}

		else
		{
			L_TRAIN(Set(0));
			R_TRAIN(Set(0));
		}
	}


	void Test()
	{
		StopMotors();
		while(IsTest() && IsEnabled())
		{
		armPosition = armEncoderLeft.GetDistance();
		SmartDashboard::PutString("DB/String 3", (string)"ArmEncoder: " + std::to_string(armEncoderLeft.GetDistance()));
		SmartDashboard::PutString("DB/String 7", (string)"WincEncoder: " + std::to_string(winch5.GetEncPosition()));
		}
	}
};

START_ROBOT_CLASS(Robot)
