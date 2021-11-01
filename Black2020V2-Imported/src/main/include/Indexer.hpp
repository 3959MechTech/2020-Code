#pragma once

#include "ctre/Phoenix.h"
#include <frc/WPILib.h>
#include "rev/CANSparkMax.h"

class Indexer
{
    private:
    //Declares the Motor controller for the indexer
        rev::CANSparkMax _fI;
        rev::CANSparkMax _mI;
        rev::CANSparkMax _bI;

    public:
        //Constructor for the Indexer class
        Indexer(int frontIndex, int midIndex, int backIndex);
        //Deconstructor for the Indexer class
        ~Indexer();

        void DirectDrive(double m1, double m2, double m3);
        void SetM1(double speed);
        void SetM2(double speed);
        void SetM3(double speed);

        void Advance(double speed);
        void Retract(double speed);

        void SendData(std::string name = "Indexer");
};