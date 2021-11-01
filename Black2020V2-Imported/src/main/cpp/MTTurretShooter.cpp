/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MTTurretShooter.h"
#include "MTMath.h"


MTTurretShooter::MTTurretShooter(int leftMotor, int rightMotor, int turretMotor, int imu): _lm(leftMotor),
    _rm(rightMotor), 
    _turretMotor(turretMotor),
    _imu(imu)
{
    _autoTrack = false;
    _threadRunning = false;
    
    _lm.Follow(_rm);

    _rm.SetInverted(false);
    _lm.SetInverted(InvertType::OpposeMaster);//could be FollowMaster
    
    _rm.Config_kF(0,.04778,0);
    _rm.Config_kP(0,.03,0);
    _rm.Config_kI(0,.0,0);
    _rm.Config_kD(0,.0,0);

    TalonSRXConfiguration turretMotorConfig;
    //pigeon is 8192 ticks per rev
    turretMotorConfig.remoteFilter0.remoteSensorSource = RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw;
    turretMotorConfig.remoteFilter0.remoteSensorDeviceID = _imu.GetDeviceNumber();

    turretMotorConfig.primaryPID.selectedFeedbackCoefficient = 3600.0/8192.0;
    turretMotorConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice::RemoteSensor0;
    turretMotorConfig.slot0.kP = 1.0;

    turretMotorConfig.motionAcceleration = 8000;
    turretMotorConfig.motionCruiseVelocity = 900;

    turretMotorConfig.neutralDeadband = .001;
    

    _turretMotor.ConfigAllSettings(turretMotorConfig);

    //Turn down un-needed data on Can Bus
    _lm.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_1_General_, 255);
    _lm.SetStatusFramePeriod(motorcontrol::StatusFrame::Status_2_Feedback0_, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_RawStatus_4_Mag, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_6_SensorFusion, 255);
    _imu.SetStatusFramePeriod(PigeonIMU_StatusFrame::PigeonIMU_CondStatus_11_GyroAccum, 255);

    _imu.SetFusedHeading(0);

}

MTTurretShooter::~MTTurretShooter()
{
    
}

void MTTurretShooter::shoot(double speed)
{
    
    
}

 void MTTurretShooter::spin(double speed)

{
    _turretMotor.Set(ControlMode::PercentOutput, speed);
}
void MTTurretShooter::setWheelSpeed(double speed)
{
    _targetWheelSpeed = speed;
    _rm.Set(ControlMode::PercentOutput, _targetWheelSpeed);
    
}

void MTTurretShooter::sendData(std::string name)
{
    frc::SmartDashboard::PutNumber(name + " heading", _imu.GetFusedHeading());

} // namespace name

void MTTurretShooter::updateTargetHeading(double degrees)
{
    _targetHeading = degrees*10.0;
}

void MTTurretShooter::track(MTPoseData drivePose)
{

}

/*
void MTTurretShooter::threadTrack()
{
    _mutex.lock();
    _threadRunning = true;
    _mutex.unlock();

    while (isAutoTracking())
    {
        track();
    }
    
    //let the world know we died
    _mutex.lock();
    _threadRunning = false;
    _mutex.unlock();

}
*/


