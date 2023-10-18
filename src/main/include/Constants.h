// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once
#include "units/length.h"
#include "units/voltage.h"
#include "units/velocity.h"
#include "units/angle.h"
#include "units/time.h"
#include "units/acceleration.h"
#include "units/angular_acceleration.h"
#include <frc/controller/ArmFeedforward.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/util/Color.h>

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

}

namespace SwerveConstants{

    enum CCUsage{
        kAllCC,
        kSensorDataOnly,
        kFaultsOnly,
        kMinimalCC
    };

    enum Usage{
        kAll,
        kPositionOnly,
        kVeclocityOnly,
        kMinimal
    };



    constexpr int BalancekP = 0;
    constexpr int BalancekI = 0;
    constexpr int BalancekD = 0;

    constexpr int CANCoderID = 12; 
    constexpr bool InvertGyro = false;

    /*Drivetrain constants*/
    constexpr double OpenLoopRamp = 0.25;
    constexpr double closedLoopRamp = 0.375;

    constexpr double DriveGearRatio = 8.14;
    // Use After Testing w/ 8.14 DGR
    // constexpr double DriveGearRatio = 6.12;
    constexpr double AngleGearRatio = 150.0 / 7.0;

    
    constexpr units::meter_t WheelCircumference{ 3.75_in * M_PI  };

    const frc::Translation2d m_FrontLeft{-9.988_in, 9.988_in};
    const frc::Translation2d m_FrontRight{9.988_in, 9.988_in};
    const frc::Translation2d m_BackLeft{-9.988_in, -9.988_in};
    const frc::Translation2d m_BackRight{9.988_in, -9.988_in};

    const frc::SwerveDriveKinematics<4> m_kinematics{m_FrontLeft,
                                               m_FrontRight,
                                               m_BackLeft,
                                               m_BackRight};




    //Change to non-linear throttle for finer tuned movements
    enum Throttle {
        LINEAR = 1,
        NONLINEAR = 2
    };


    //Tip Correction PID (PITCH)
    constexpr double PitchKP = 0.1;
    constexpr double PitchKD = 0.0;


    //Tip Correction PID (ROLL) 
    constexpr double RollKP = 0.1;
    constexpr double RollKD = 0.0;

    constexpr units::second_t Timeout {0.1};
    
    /*setting up correct units for the simepleMotorFeedforward KS gain*/
    constexpr units::volt_t DriveKS{0.0};

    constexpr units::volt_t VoltageKV{0.0};
    constexpr units::foot_t FeetKV{0.0};
    constexpr units::second_t TimeKV{0.0};
    /*Setting up correct units for the simpleMotorFeedforward KV gain
    Change VoltageKV when wanting to change 
    the KV gain*/
    // constexpr auto DriveKV = VoltageKV * TimeKV / FeetKV;
    constexpr auto DriveKV = FeetKV.value() == 0.0 ? VoltageKV * TimeKV / units::foot_t{0.01} : VoltageKV * TimeKV / FeetKV;

    constexpr units::volt_t VoltageKA{0.0};
    constexpr units::foot_t FeetKA{0.0};
    constexpr units::second_t TimeKA{0.0};
    /*Setting up correct units for the simpleMotorFeedforward KA gain
    Change VoltageKA when wanting to change the KA gain*/
    // constexpr auto DriveKA = VoltageKA * (TimeKA * TimeKA) / FeetKA; 
    constexpr auto DriveKA = FeetKA.value() == 0.0 ? VoltageKA * (TimeKA * TimeKA) / units::foot_t{0.01} : VoltageKA * (TimeKA * TimeKA) / FeetKA;
    constexpr units::volt_t kNominal {12.0};
    constexpr double kNominalDouble = 12.0;

    /*Angle Encoder Invert*/
    constexpr bool CanCoderInvert = false;

    /*Swerve Angle Motor PID gains*/
    constexpr double AngleKP = 0.15;
    constexpr double AngleKI = 0.0;
    constexpr double AngleKD = 0.00;
    constexpr double AngleKF = 0.0;

    /*Swerve Angle Current Limit Config*/
    constexpr bool AngleEnableCurrentLimit = true;
    constexpr int AngleContinuousCurrentLimit = 10;
    constexpr int AnglePeakCurrentLimit = 20;
    constexpr double AnglePeakCurrentDuration = 0.1;
    
    /*Swerve Drive Motor PID gains*/
    constexpr double DriveKP = 0.1;
    constexpr double DriveKI = 0.0;
    constexpr double DriveKD = 0.00;
    constexpr double DriveKF = 0.0;

    /*Swerve Drive Current Limit Config*/
    constexpr bool DriveEnableCurrentLimit = true;
    constexpr int DriveContinuousCurrentLimit = 30;
    constexpr int DrivePeakCurrentLimit = 40;
    constexpr double DrivePeakCurrentDuration = 0.1;

    /*Motor Inverts Config*/
    constexpr bool AngleMotorInvert = true;
    constexpr bool DriveMotorInvert = false;

    /* Swerve Profiling values */
    constexpr units::meters_per_second_t MaxSpeed{1.0};
    constexpr units::degrees_per_second_t MaxAngularVelocity{360.0 * 0.25};
    constexpr bool IsFieldRelative = false;
    constexpr bool IsOpenLoop = false;  

}

namespace DPAD{

    enum ORIENTATION{
        FRONT = 0,
        FRONT_RIGHT = 45,
        RIGHT = 90,
        DOWN_RIGHT = 135,
        DOWN = 180,
        DOWN_LEFT,
        LEFT = 270,
        FRONT_LEFT = 315,
        NON_ORIENTED = -1
    };

    enum NODE_LEVEL{
        HIGH = 0,
        MID = 270,
        LOW = 180,
        NON_SPECIFIED = -1
    };
};

namespace FrontLeftModule{
    constexpr int DriveMotorID = 3;
    constexpr int AngleMotorID = 4;
    constexpr int CanCoderID = 2;
    // constexpr double AngleOffset = 352.617;
    constexpr double AngleOffset = 205;
    const double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset };
}

namespace FrontRightModule{
    constexpr int DriveMotorID = 9;
    constexpr int AngleMotorID = 2;
    constexpr int CanCoderID = 5;
    // constexpr double AngleOffset = 321.855;
    constexpr double AngleOffset = 290;
    const double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset};
}

namespace BackLeftModule{
    constexpr int DriveMotorID = 5;
    constexpr int AngleMotorID = 6;
    constexpr int CanCoderID = 8;
    // constexpr double AngleOffset = 97.822;
    constexpr double AngleOffset = 49;
    constexpr double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset};
}

namespace BackRightModule{
    constexpr int DriveMotorID = 7;
    constexpr int AngleMotorID = 8;
    constexpr int CanCoderID = 11;
    // constexpr double AngleOffset = 71.104;
    constexpr double AngleOffset = 41;
    const double Constants[4] = { DriveMotorID, AngleMotorID, CanCoderID, AngleOffset};
}

namespace AutoConstants{
    constexpr units::meters_per_second_t MaxSpeed{ 1 };
    constexpr units::meters_per_second_squared_t MaxAccel{ 1 };
    constexpr units::radians_per_second_t MaxAngularSpeed{ 6 };
    constexpr units::radians_per_second_squared_t MaxAngularAccel{ 3 };

    /*Auto Swerve Drive Motor PID gains*/
    constexpr double XDriveKP = 1.0;
    constexpr double XDriveKD = 0.0;

    const frc2::PIDController XPID{ XDriveKP, 0.0, XDriveKD };
    
    constexpr double YDriveKP = 1.0;
    constexpr double YDriveKD = 0.0;
    
    const frc2::PIDController YPID{ YDriveKP, 0.0, YDriveKD };

    /* Auto Swerve Angle Motor PID gains*/  
    constexpr double AngleKP = 3;
    constexpr double AngleKD = 0.0;

    const frc2::PIDController ZPID{ AngleKP, 0.0, AngleKD };

    const frc::ProfiledPIDController<units::radian> ThetaPID{AngleKP, 
                                              0.0, 
                                              AngleKD, 
                                              frc::TrapezoidProfile<units::radian>::Constraints{MaxAngularSpeed,
                                                                                                 MaxAngularAccel}};
}

namespace ElevatorConstants
{
    constexpr int kElevatorID = 10;
    constexpr int kElevatorFollowerID = 11;
    constexpr int kEncoderID = 0;
   
    constexpr double kElevatorStow = 0;
    constexpr double kElevatorScoreMidCube = 24.951;
    constexpr double kElevatorScoreMidCone = 24.951;
    constexpr double kElevatorScoreHighCube = 44.265;
    constexpr double kElevatorScoreHighCone = 53.697;
    constexpr double kElevatorScoreHybrid = 0;
    constexpr double kElevatorIntakeDoubleSubstation = 0;
    // constexpr double kElevatorMotionAcceleration = 0;
    // constexpr double kElevatorCruiseVelocity = 0;
    constexpr double kElevatorP = 0.1;
    constexpr double kElevatorI = 0;
    constexpr double kElevatorD = 0.0005;
    constexpr double kBufferZone = 0;
    constexpr units::time::second_t TargetTime{0.5};

    constexpr double kManualSpeed = 1;
    constexpr double ElevatorThreshold = -14.0; //this prob needa be changed
    constexpr double MaxTimeAllowed = 0.0; //change this
    
}

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

    constexpr double HighConePos = 14.91;//125.81
    //needs defining
    constexpr double HighCubePos = 7.96;//67.16
    constexpr double MidConePos = 9.96;//84.03
    constexpr double MidCubePos = 9.96;
    constexpr double SubstationPos = 0.0;//fix
    constexpr double GroundIntake = 0.0;//fix
    // constexpr double WristZero = -0.5;//fix    !!!this is star_pos
}

namespace IntakeConstants{
    enum intakeState{
    noGamepiece,
    hasCone,
    hasCube,
    coneIntaking, 
    coneOuttaking, 
    cubeIntaking, 
    cubeOuttaking
  };
  constexpr double normalOutputCurrent = 30.0;
  constexpr int IntakeMotorID = 13;
  constexpr int IntakeMotorCurrentLimit = 50;
}

namespace LEDConstants{
    enum LEDState{
        Standby, Yellow, Purple, Error, Off
    };
    constexpr units::second_t Timeout{ 10.0 };
    
    constexpr int CANdleID = 20;

    constexpr units::second_t IntakeFlashTime{1.0};
}

namespace ControllerConstants{
    constexpr double Deadband = 0.1;
    constexpr double TriggerActivate = 0.8;
    
    constexpr int XboxDriveID = 0;
    constexpr int XboxHaperatorID = 1;

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
    constexpr int xboxRightDPad = 11;
}
