#include "MechTechDifferential.hpp"

#include "MTMath.h"

using namespace frc;
//Constructor for the Drivetrain class
MTDifferential::MTDifferential(int rightmaster, int rightslave, int leftmaster, int leftslave, int imu, int solenoid) : _rm(rightmaster), 
    _rs(rightslave), _lm(leftmaster), _ls(leftslave), _imu(imu), _shifter(solenoid)
{
    _wheelDiameter = 6;//6 inch wheel diameter
    _encoderTick = 2048;//Encoder tick per revolution of motor
    _wheelBase = 27;// Size of drive base from left to right
    _baseLength = 33;//Size of drive base from front to back

    _gearRatioL = 20.83;
    _gearRatioH = 9.167;

    _rEncoderValue = 0;
    _lEncoderValue = 0;
    _headingOffset = 0;

    UpdateTickScalers();

    SupplyCurrentLimitConfiguration currentLimit;
    currentLimit.enable = true;
    currentLimit.triggerThresholdCurrent = 15.0;
    currentLimit.currentLimit = 10.0;

    _lm.ConfigSupplyCurrentLimit(currentLimit);
    _rm.ConfigSupplyCurrentLimit(currentLimit);

    _ls.ConfigSupplyCurrentLimit(currentLimit);
    _rs.ConfigSupplyCurrentLimit(currentLimit);

    _rs.Follow(_rm);
    _ls.Follow(_lm);
    _rm.SetInverted(false);
    _lm.SetInverted(true);
    _rs.SetInverted(InvertType::FollowMaster);
    _ls.SetInverted(InvertType::FollowMaster);


    _lm.Config_kF(0,.045);
    _rm.Config_kF(0,.045);

    _lm.Config_kP(0,.15);//.15
    _rm.Config_kP(0,.15);

    _lm.Config_kI(0,.0);
    _rm.Config_kI(0,.0);

    _lm.Config_kD(0,.0);
    _rm.Config_kD(0,.0);

    _lm.SelectProfileSlot(0,0);
    _rm.SelectProfileSlot(0,0);

    //Turn down un-needed data on Can Bus
    _ls.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_1_General_, 255);
    _ls.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_2_Feedback0_, 255);
    _rs.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_1_General_, 255);
    _rs.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_2_Feedback0_, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_RawStatus_4_Mag, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_6_SensorFusion, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum, 255);



    _highGear = true;
    _shifter.Set(_highGear);

    _poseTimer.Start();

}

//Deconstructor for the Drivetrain class
MTDifferential::~MTDifferential()
{

}

void MTDifferential::ResetEncoders()
{
    _lm.SetSelectedSensorPosition(0,0,0);
    _rm.SetSelectedSensorPosition(0,0,0);
}

void MTDifferential::SetSpeed(double leftSpeed, double rightSpeed)
{
    _lm.Set(ControlMode::PercentOutput, leftSpeed);
    _rm.Set(ControlMode::PercentOutput, rightSpeed);
}

void MTDifferential::SetSpeed(ControlMode mode ,double leftSpeed, double rightSpeed)
{
    _lm.Set(mode, leftSpeed);
    _rm.Set(mode, rightSpeed);
}
void MTDifferential::ArcadeDrive(ControlMode mode ,double transVel, double rotVel)
{
    rotVel = rotVel/2.0;
    _lm.Set(mode, (transVel-rotVel)/2.0);
    _rm.Set(mode, (transVel+rotVel)/2.0);

}

void MTDifferential::Shift(bool high)
{
    if(_highGear == high)
    {
        return;
    }
    _highGear = high;
    _shifter.Set(_highGear);
}

void MTDifferential::StartMotionProfile(BufferedTrajectoryPointStream& leftStream, BufferedTrajectoryPointStream& rightStream, ControlMode mode )
{
    _lm.StartMotionProfile(leftStream, 10, mode);
    _rm.StartMotionProfile(rightStream, 10, mode);
}

bool MTDifferential::IsMotionProfileFinished()
{
    if(_lm.IsMotionProfileFinished() && _rm.IsMotionProfileFinished())
    {
        return true;
    }
    return false;
}

void MTDifferential::configMotors(TalonFXConfiguration leftMotor, TalonFXConfiguration rightMotor)
{
    _lm.ConfigAllSettings(leftMotor);
    _rm.ConfigAllSettings(rightMotor);
}


void MTDifferential::SetGearRatio(double ratioLow, double ratioHigh)
{
    _gearRatioL = ratioLow;
    _gearRatioH = ratioHigh;
    UpdateTickScalers();
}

void MTDifferential::SetMaxVel(double maxVelLow, double maxVelHigh)
{
    _maxVelLow = maxVelLow;
    _maxVelHigh = maxVelHigh;
    UpdateTickScalers();
}
void MTDifferential::SetDriveTrainData(double wheelDiameter, double wheelBase, double length)
{
    _wheelDiameter = wheelDiameter;
    _wheelBase = wheelBase;
    _baseLength = length;
    UpdateTickScalers();
}

void MTDifferential::UpdateTickScalers()
{
    _inchPerTickHigh = MTMath::InchPerTicks(_wheelDiameter,_gearRatioH, _encoderTick);
    _inchPerTickLow = MTMath::InchPerTicks(_wheelDiameter,_gearRatioL, _encoderTick);
    _ticksPerInchHigh = MTMath::TicksPerInch(_wheelDiameter, _gearRatioH, _encoderTick);
    _ticksPerInchLow = MTMath::TicksPerInch(_wheelDiameter, _gearRatioL, _encoderTick);
    
    //_ticksPerInch = (ticksPerMotorRev*gearRatio)/(wheelDiameter*3.14159);
}
void MTDifferential::SetHeadingOffset(double radians)
{
    _headingOffset = radians;
}

bool MTDifferential::isHighGear()
{
    if (_gearRatioL)
    {
        return true;
    }
        return false;
}

double MTDifferential::GetLeftEncoderPosition()
{
    return _lm.GetSelectedSensorPosition(0);
}

double MTDifferential::GetRightEncoderPosition()
{
    return _rm.GetSelectedSensorPosition(0);
}

void MTDifferential::UpdatePose()
{
    MTPoseData lastPose = _pose.Get();
    MTPoseData newPose;

    newPose.timestamp = _poseTimer.Get();

    //get current heading
    newPose.phi = MTMath::CleanAngle( MTMath::DegreesToRadians(_imu.GetFusedHeading()));
    newPose.phi = MTMath::CleanAngle(newPose.phi-_headingOffset);

    //update x and y position as long as we aren't doing a 0 point turn
    double l,r,d, tickScaler;
    if(_highGear)
    {
        tickScaler = _inchPerTickHigh;
    }else
    {
        tickScaler = _inchPerTickLow;
    }

    l = _lm.GetSelectedSensorPosition()*tickScaler;
    r = _rm.GetSelectedSensorPosition()*tickScaler;
    d = ((l-_lEncoderValue)+(r-_rEncoderValue))/2.0;
    //update position based on heading and distance moved.
    newPose.x = lastPose.x + d*cos(newPose.phi);
    newPose.y = lastPose.y + d*sin(newPose.phi);
    
    //store current l and r into old values to be removed next time
    _lEncoderValue = l;
    _rEncoderValue = r;

    double vl = _lm.GetSelectedSensorVelocity()*10.0;//convert from ticks per 100ms to ticks per s
    double vr = _rm.GetSelectedSensorVelocity()*10.0;//convert from ticks per 100ms to ticks per s
    
    newPose.vx = (tickScaler*(vl+vr)/2.0)*cos(newPose.phi);
    newPose.vy = (tickScaler*(vl+vr)/2.0)*sin(newPose.phi);
    newPose.w = (newPose.phi-lastPose.phi)/(newPose.timestamp-lastPose.timestamp);

    newPose.ax = (newPose.vx-lastPose.vx)/(newPose.timestamp-lastPose.timestamp);
    newPose.ay = (newPose.vy-lastPose.vy)/(newPose.timestamp-lastPose.timestamp);
    newPose.aa = (newPose.w-lastPose.w)/(newPose.timestamp-lastPose.timestamp);

    SetPose(newPose);

/*//old 2018 code
    //grab euler angles from imu and store in member vector.
		eulers = imu.GetVector(BNO055::vector_type_t::VECTOR_EULER);

		//convert yaw angle to radians
		double phi = -CleanAngle(3.14159*eulers.x/180.0);
		phi = CleanAngle(phi-headingOffset); //apply offset

		double l,r,d;
		//update x and y position as long as we aren't doing a 0 point turn
		if(!turning)
		{
			//get the distance traveled by each wheel
			l = 3.14159*4.0*(drive.GetLeftEncoderPosition()/1440.0);
			r = 3.14159*4.0*(drive.GetRightEncoderPosition()/1440.0);

			//remove old position, so we are only adding the change in distance
			//find the distance the CENTER of the robot moved
			d = ((l-oldLDrivePos)+(r-oldRDrivePos))/2.0;

			if(useSimData) //used to simulate robot moving
			{
				drivePos.SetX(simx);
				drivePos.SetY(simy);
			}else
			{
				//update position based on heading and distance moved.
				drivePos.SetX( drivePos.GetX() + d*cos(phi));
				drivePos.SetY(drivePos.GetY() + d*sin(phi));
			}
			//store current l and r into old values to be removed next time
			oldLDrivePos = l;
			oldRDrivePos = r;
		}else
		{
			drive.ResetEncoders();
			oldLDrivePos = drive.GetLeftEncoderPosition();
			oldRDrivePos = drive.GetRightEncoderPosition();
		}


		drivePos.SetPhi(phi);
*/
}

MTPoseData MTDifferential::GetPose()
{
    return _pose.Get();
}

void MTDifferential::SetPose(MTPoseData pose)
{
    _pose.Update(pose);
}

void MTDifferential::SendData()
{

    MTPoseData pos = GetPose();

    frc::SmartDashboard::PutNumber("Drive Pose X", pos.x);
    frc::SmartDashboard::PutNumber("Drive Pose Y", pos.y);
    frc::SmartDashboard::PutNumber("Drive Pose phi", pos.phi);
    frc::SmartDashboard::PutNumber("Drive Pose vX", pos.vx);
    frc::SmartDashboard::PutNumber("Drive Pose vY", pos.vy);
    frc::SmartDashboard::PutNumber("Drive Pose w", pos.w);
    frc::SmartDashboard::PutNumber("Drive Pose AX", pos.ax);
    frc::SmartDashboard::PutNumber("Drive Pose AY", pos.ay);
    frc::SmartDashboard::PutNumber("Drive Pose AA", pos.aa);
    frc::SmartDashboard::PutNumber("Drive Pose timestamp", pos.timestamp);

    
    frc::SmartDashboard::PutNumber("Drive IMU Heading", _imu.GetFusedHeading());

    frc::SmartDashboard::PutNumber("Left Drive Motor Speed", _lm.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Right Drive Motor Speed", _rm.GetSelectedSensorVelocity());
    frc::SmartDashboard::PutNumber("Left Drive Motor Position", _lm.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Right Drive Motor Position", _rm.GetSelectedSensorPosition());
    frc::SmartDashboard::PutNumber("Left Drive Motor Temp (C)", _lm.GetTemperature());
    frc::SmartDashboard::PutNumber("Right Drive Motor Temp (C)", _rm.GetTemperature());

    frc::SmartDashboard::PutNumber("ControlMode", (int)_lm.GetControlMode());
    frc::SmartDashboard::PutBoolean("Left Path Finished", _lm.IsMotionProfileFinished());
    frc::SmartDashboard::PutBoolean("Right Path Finished", _rm.IsMotionProfileFinished());

    if( _lm.GetControlMode() == ControlMode::Velocity ||
        _lm.GetControlMode() == ControlMode::Position ||
        _lm.GetControlMode() == ControlMode::MotionProfile ||
        _lm.GetControlMode() == ControlMode::MotionProfileArc ||
        _lm.GetControlMode() == ControlMode::MotionMagic
        )
    {
        frc::SmartDashboard::PutNumber("Left PID Error", _lm.GetClosedLoopError());
        frc::SmartDashboard::PutNumber("Right PID Error", _rm.GetClosedLoopError());
        frc::SmartDashboard::PutNumber("Left Derivative Error", _lm.GetErrorDerivative());
        frc::SmartDashboard::PutNumber("Right Derivative Error", _rm.GetErrorDerivative());
        frc::SmartDashboard::PutNumber("Left Intergral Error", _lm.GetIntegralAccumulator());
        frc::SmartDashboard::PutNumber("Right Intergral Error", _rm.GetIntegralAccumulator());
    }else
    {
        frc::SmartDashboard::PutNumber("Left PID Error", 0);
        frc::SmartDashboard::PutNumber("Right PID Error", 0);
        frc::SmartDashboard::PutNumber("Left Derivative Error", 0);
        frc::SmartDashboard::PutNumber("Right Derivative Error", 0);
        frc::SmartDashboard::PutNumber("Left Intergral Error", 0);
        frc::SmartDashboard::PutNumber("Right Intergral Error", 0);
    }
    frc::SmartDashboard::PutBoolean("Path Finished", IsMotionProfileFinished());
}

