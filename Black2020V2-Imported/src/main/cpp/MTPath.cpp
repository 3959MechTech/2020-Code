/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "MTPath.h"

MTPath::MTPath(double wheelDiameter, double ticksPerMotorRev, double gearRatio)
{
    _ticksPerInch = (ticksPerMotorRev*gearRatio)/(wheelDiameter*M_PI);
}



/*
    bool forward = true; // set to false to drive in opposite direction of profile (not really needed
                         // since you can use negative numbers in profile).

*/
void MTPath::GetStreamFromArray(BufferedTrajectoryPointStream& stream, const double traj[][5], int trajLen, bool left, bool forward = true)
{

    TrajectoryPoint point; // temp for for loop, since unused params are initialized
                           // automatically, you can alloc just one
    MTPath::column pos, vel;
    if(left)
    {
        pos = MTPath::column::position_left;
        vel = MTPath::column::velocity_left;
    }else
    {
        pos = MTPath::column::position_right;
        vel = MTPath::column::velocity_right;
    }
    
    /* clear the buffer, in case it was used elsewhere */
    stream.Clear();
    
    /* Insert every point into buffer, no limit on size */
    for (int i = 0; i < trajLen; ++i) {

        double direction = forward ? +1 : -1;
        double position = traj[i][pos];
        double velocity = traj[i][vel];
        int durationMilliseconds = traj[i][MTPath::column::duration];

        /* for each point, fill our structure and pass it to API */
        point.timeDur = durationMilliseconds;
        point.position = direction * position * _ticksPerInch; // Convert Revolutions to
                                                         // Units
        point.velocity = direction * velocity * _ticksPerInch / 600.0; // Convert RPM to
                                                                 // Units/100ms
        point.auxiliaryPos = 0;
        point.auxiliaryVel = 0;
        point.profileSlotSelect0 = 0; /* which set of gains would you like to use [0,3]? */
        point.profileSlotSelect1 = 0; /* which set of gains would you like to use [0,3]? */
        point.zeroPos = (i == 0); /* set this to true on the first point */
        point.isLastPoint = ((i + 1) == trajLen); /* set this to true on the last point */
        point.arbFeedFwd = 0; /* you can add a constant offset to add to PID[0] output here */

        stream.Write(point);

        
    }
}
