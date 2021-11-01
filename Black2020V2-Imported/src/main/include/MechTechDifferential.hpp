

#include "ctre/Phoenix.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/Solenoid.h>
#include <frc/Timer.h>

#include "MTPose.hpp"


class MTDifferential
{
    private:
        //Declares the 4 Falcon motors
        TalonFX _rm,
                _rs,
                _lm,
                _ls;
        PigeonIMU _imu;
        
        MTPose _pose;

        frc::Solenoid _shifter;

        frc::Timer _poseTimer{};
        
        double _wheelDiameter;//Size of wheel
        double _wheelBase;//distance from left to right
        double _baseLength;//distance from front to back

        double _encoderTick;//encoder ticks per rev of motor
        double _rEncoderValue;//latest values of encoders
        double _lEncoderValue;//latest values of encoders

        double _gearRatioL;//gear ratio of gear box in low
        double _gearRatioH;//gear ratio of gear box in high

        double _inchPerTickHigh;
        double _inchPerTickLow;
        double _ticksPerInchHigh;
        double _ticksPerInchLow;

        bool _highGear;//are we in high gear

        double _maxVelHigh;//max velocity in high gear
        double _maxVelLow;//max velocity in low gear:

        double _headingOffset;



    public:
        //Constructor and Destructor classes for the Mech tech Differential drive
        MTDifferential(int rightmaster, int rightslave, int leftmaster, int leftslave, int imu, int shifter);
        ~MTDifferential();

        void UpdatePose();
        MTPoseData GetPose();
        void SetPose(MTPoseData pose);

        void SetHeadingOffset(double radians);
        
        void SetSpeed(double leftSpeed, double rightSpeed);
        void SetSpeed(ControlMode mode ,double leftSpeed, double rightSpeed);
        void ArcadeDrive(ControlMode mode ,double transVel, double rotVel);

        void StartMotionProfile(BufferedTrajectoryPointStream& leftStream, BufferedTrajectoryPointStream& rightStream, ControlMode mode);
        bool IsMotionProfileFinished();

        void ResetEncoders();
        void SendData();

	    void configMotors(TalonFXConfiguration leftMotor, TalonFXConfiguration rightMotor);

        void SetGearRatio(double ratioLow, double ratioHigh);
        void SetMaxVel(double maxVelLow, double maxVelHigh);
        void SetDriveTrainData(double wheelDiameter, double wheelBase, double length);
        void UpdateTickScalers();

        void Shift(bool high = false);
        bool isHighGear();

        double GetLeftEncoderPosition();
        double GetRightEncoderPosition();

        void ConfigRobot(double diameter, double base, double length, double ticks)
        {
            _wheelDiameter = diameter;
            _wheelBase = base;
            _baseLength = length;
            _encoderTick = ticks;
        }


};