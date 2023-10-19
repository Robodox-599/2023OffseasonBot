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
    constexpr int xboxLXAxis = 0;
    constexpr int xboxLYAxis = 1;
    constexpr int xboxRXAxis = 4;
    constexpr int xboxRYAxis = 5;
    constexpr int ManualThreshold = 0.02;
    constexpr int xboxLTAxis = 2;
    constexpr int xboxRTAxis = 3;


}  // namespace OperatorConstants

namespace WristConstants {
    constexpr double kWristI = 0.0;
    constexpr double kWristP = 0.05;
    constexpr double kWristD = 0.0005;
    constexpr double start_pos = -0.5;// hard stop pos
    constexpr double bufferZone = 0.0;//still needs defining
    constexpr double stepperConstant = 0.1;
    constexpr units::time::second_t TargetTime{0.1};
    constexpr int motorID = 12;
    constexpr double WristThreshold = -14.0;
}
namespace WristPositions{
    constexpr double HighConePos = 14.91;//125.81
    //needs defining
    constexpr double HighCubePos = 7.96;//67.16
    constexpr double MidConePos = 9.96;//84.03
    constexpr double MidCubePos = 9.96;
    constexpr double SubstationPos = 10.9048;//fix
    constexpr double GroundIntake = 16.809;//fix
    // constexpr double WristZero = -0.5;//fix    !!!this is star_pos
}
