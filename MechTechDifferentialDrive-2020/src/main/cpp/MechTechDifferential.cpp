#include "MechTechDifferential.hpp"

using namespace frc;
//Constructor for the Drivetrain class
MTDifferential::MTDifferential(int rightmaster, int rightslave, int leftmaster, int leftslave) : _rm(rightmaster), _rs(rightslave), _lm(leftmaster), _ls(leftslave)
{
    _wheelDiameter = 6;//6 inch wheel diameter
    _encoderTick = 2048;//Encoder tick per revolution of motor
    _wheelBase = 24;// Size of drive base from left to right
    _baseLength = 30;//Size of drive base from front to back

    _gearRatioL = 20.83;
    _gearRatioH = 9.167;

    _rEncoderValue = 0;
    _lEncoderValue = 0;
}

//Deconstructor for the Drivetrain class
MTDifferential::~MTDifferential()
{

}
/*
double MTDifferential::GetRightEncoderPosition()
{
return _rm.GetSelectedSensorPosition(0);
}
*/