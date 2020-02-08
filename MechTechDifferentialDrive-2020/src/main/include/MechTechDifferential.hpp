#include "ctre/Phoenix.h"
#include <frc/WPILib.h>

class MTDifferential
{
    private:
        //Declares the 4 Falcon motors
        TalonFX _rm,
                _rs,
                _lm,
                _ls;
        
        double _wheelDiameter;//Size of wheel
        double _wheelBase;//distance from left to right
        double _baseLength;//distance from front to back

        double _encoderTick;//encoder ticks per rev of motor
        double _rEncoderValue;//latest values of encoders
        double _lEncoderValue;//latest values of encoders

        double _gearRatioL;//gear ratio of gear box in low
        double _gearRatioH;//gear ratio of gear box in high

        bool _highGear;//are we in high gear

        double _maxVelHigh;//max velocity in high gear
        double _maxVelLow;//max velocity in low gear:

    public:
        //Constructor and Destructor classes for the Mech tech Differential drive
        MTDifferential(int rightmaster, int rightslave, int leftmaster, int leftslave);
        ~MTDifferential();
};