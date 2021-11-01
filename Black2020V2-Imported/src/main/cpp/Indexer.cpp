
#include "Indexer.hpp"

using namespace frc;
//Constructor for the Indexer Class
Indexer::Indexer(int frontIndex, int midIndex, int backIndex) : _fI(frontIndex, rev::CANSparkMax::MotorType::kBrushless), 
    _mI(midIndex, rev::CANSparkMax::MotorType::kBrushless), 
    _bI(backIndex, rev::CANSparkMax::MotorType::kBrushless)
{

    _fI.RestoreFactoryDefaults();
    _mI.RestoreFactoryDefaults();
    _bI.RestoreFactoryDefaults();
    
    _fI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 50);
    _fI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 50);
    _fI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);
    _mI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 50);
    _mI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 50);
    _mI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);
    _bI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 50);
    _bI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 50);
    _bI.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 50);
}

//Deconstructor for the Indexer class
Indexer::~Indexer()
{

}

void Indexer::Advance(double speed)
{
    _fI.Set(speed);
    _mI.Set(speed);
    _bI.Set(speed);
}

void Indexer::Retract(double speed)
{
    _fI.Set(-speed);
    _mI.Set(-speed);
    _bI.Set(-speed);
}

void Indexer::DirectDrive(double m1, double m2, double m3)
{
    _fI.Set(m1);
    _mI.Set(m2);
    _bI.Set(m3);
}

void Indexer::SetM1(double speed)
{
    _fI.Set(speed);
}

void Indexer::SetM2(double speed)
{
    _mI.Set(speed);
}

void Indexer::SetM3(double speed)
{
    _bI.Set(speed);
}