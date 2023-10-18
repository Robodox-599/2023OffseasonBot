// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "units/angle.h"
#include <units/math.h>
#include <units/length.h>
#include <units/voltage.h>
#include <units/velocity.h>
#include <units/time.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants.  This should not be used for any other
 * purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace OperatorConstants {

constexpr int kDriverControllerPort = 1;

}  // namespace OperatorConstants

namespace ElevatorConstants
{
    constexpr int kElevatorID = 9;
    constexpr int kElevatorFollowerID = 4;
   
    constexpr int kElevatorStow = 0;
    constexpr int kElevatorScoreMidCube = 24.951;
    constexpr int kElevatorScoreMidCone = 24.951;
    constexpr int kElevatorScoreHighCube = 44.265;
    constexpr int kElevatorScoreHighCone = 53.697;
    constexpr int kElevatorScoreHybrid = 0;
    constexpr int kElevatorScoreDoubleSubstation = 0;
    // constexpr int kElevatorMotionAcceleration = 0;
    // constexpr int kElevatorCruiseVelocity = 0;
    constexpr int kElevatorP = 0.1;
    constexpr int kElevatorI = 0;
    constexpr int kElevatorD = 0.0005;
    constexpr int kEncoderID = 0;
    constexpr int kBufferZone = 0;
    constexpr units::time::second_t TargetTime{0.5};

    constexpr double kManualSpeed = 1;
    constexpr double ElevatorThreshold = -14.0; //this prob needa be changed
    constexpr double MaxTimeAllowed = 0.0; //change this
    
    }

namespace ControllerConstants
{
    constexpr double Deadband = 0.5;
    constexpr double TriggerActivate = 0;

    constexpr int XboxDriveID = 1;
    //constexpr int XboxOperatorID = 1;

    constexpr int xboxLXAxis = 0;
    constexpr int xboxLYAxis = 1;
    constexpr int xboxRXAxis = 4;
    constexpr int xboxRYAxis = 5;

    constexpr int xboxLTAxis = 2;
    constexpr int xboxRTAxis = 3;

    constexpr int xboxA = 1;
    constexpr int xboxB = 2;
    constexpr int xboxX = 3;
    constexpr int xboxY = 4;
    constexpr int xboxLB = 5;
    constexpr int xboxRB = 6;
    constexpr int xboxView = 7;
    constexpr int xboxMenu = 8;
    constexpr int xboxLeftJoyPress = 9;
    constexpr int xboxRightJoyPress = 10;
    //constexpr int xboxRightDPad = 0;
}
