#include "Periscope.h"
#include <ctre/phoenix.h>
#include <frc/WPILib.h>
using namespace frc;

Periscope::Periscope(int leftM, int rightM): _left(leftM), _right(rightM)
{
//Set these when we have a motor to run them
    posVals[Home][0] = 0;//Position Value
	posVals[Home][1] = 0;//Ramp
	posVals[Home][2] = 0;//Max Speed

	posVals[Low][0] = 0;
	posVals[Low][1] = 0;
	posVals[Low][2] = 0;

	posVals[Level][0] = 0;
	posVals[Level][1] = 0;
	posVals[Level][2] = 0;

    posVals[High][0] = 0;
	posVals[High][1] = 0;
	posVals[High][2] = 0;
//Also Set these when we have a motor to run them

	TalonSRXConfiguration left, right;

	left.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Relative;
	left.slot0.kP = .3;
	left.slot0.kI = 0;
	left.slot0.kD = 0;
	left.slot0.kF = 0;
	left.motionAcceleration = 1000;
    left.motionCruiseVelocity = 300;
    left.neutralDeadband = .001;
	left.reverseSoftLimitEnable = true;
	

	right.primaryPID.selectedFeedbackSensor = FeedbackDevice::CTRE_MagEncoder_Relative;
	right.slot0.kP = .3;
	right.slot0.kI = 0;
	right.slot0.kD = 0;
	right.slot0.kF = 0;
	right.motionAcceleration = 1000;
    right.motionCruiseVelocity = 300;
    right.neutralDeadband = .001;
	right.reverseSoftLimitEnable = true;
	
	ConfigureLeft(left);
	ConfigureRight(right);

	_left.SetSensorPhase(false);
	_left.SetInverted(false);
	_right.SetSensorPhase(false);
	_right.SetInverted(false);


    
}

void Periscope::ConfigureLeft(TalonSRXConfiguration left)
{
	_left.ConfigAllSettings(left);
}

void Periscope::ConfigureRight(TalonSRXConfiguration right)
{
	_right.ConfigAllSettings(right);
}

void Periscope::SetMotorSpeed(double speed)
{
         _left.Set(ControlMode::PercentOutput,speed);
		 _right.Set(ControlMode::PercentOutput,speed);
}

void Periscope::SetPPos(PPos pos)
{
	if(pos != current)
	{
		current = pos;
		if(posVals[pos][0]<=_left.GetSelectedSensorPosition(0))
		{
			_left.SelectProfileSlot(0,0);

		}else
		{
			_left.SelectProfileSlot(0,0);
		}
		_left.Set(ControlMode::Position, posVals[pos][0]);
	}
	if(pos != current)
	{
		current = pos;
		if(posVals[pos][0]<=_right.GetSelectedSensorPosition(0))
		{
			_left.SelectProfileSlot(0,0);

		}else
		{
			_right.SelectProfileSlot(0,0);
		}
		_right.Set(ControlMode::Position, posVals[pos][0]);
	}
}



