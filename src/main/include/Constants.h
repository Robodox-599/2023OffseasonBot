// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

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

constexpr int kDriverControllerPort = 0;
constexpr int XboxLYAxis=0;
constexpr int XboxLXAxis=1;
} 
namespace MotorConstants{
constexpr int LBMotorID = 0;
constexpr int RBMotorID = 1;
constexpr int LFMotorID = 2;
constexpr int RFMotorID = 3;
constexpr double MotorP= 0.0;
constexpr double MotorI=0.0;
constexpr double MotorD=0.0;
constexpr double BufferZone=0.5;
constexpr double Deadband=0.05;
constexpr double TurnGearRatio=5;
constexpr double DriveGearRation=5;
constexpr double PercentMaxOutput=0.6;//for drive motors

} // namespace OperatorConstants
