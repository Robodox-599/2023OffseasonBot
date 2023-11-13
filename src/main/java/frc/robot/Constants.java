// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ControllerConstants
  {
    public static final double deadband = 0.1;
    public static final double triggerActivate = 0.8;
    
    //public static final int driveRadID = 0;
    public static final int xboxHaperatorID = 1;

    public static final int xboxLXAxis = 0;
    public static final int xboxLYAxis = 1;
    public static final int xboxRXAxis = 4;
    public static final int xboxRYAxis = 5;

    public static final int xboxLTAxis = 2;
    public static final int xboxRTAxis = 3;

    public static final int xboxA = 1;
    public static final int xboxB = 2;
    public static final int xboxX = 3;
    public static final int xboxY = 4;
    public static final int xboxLB = 5;
    public static final int xboxRB = 6;
    public static final int xboxView = 7;
    public static final int xboxMenu = 8;
    public static final int xboxLeftJoyPress = 9;
    public static final int xboxRightJoyPress = 10;
    public static final int xboxRightDPad = 11;
  }

  public static final class ElevatorConstants
  {
    public static final int kElevatorID = 9;
    public static final int kElevatorFollowerID = 4;
    public static final int kEncoderID = 0;
   
    public static final double kElevatorStow = 0;

    public static final double kElevatorScoreMidCube = 24.951/4;
    public static final double kElevatorScoreMidCone = 24.951/4;
    public static final double kElevatorScoreHighCube = 44.265/4;
    public static final double kElevatorScoreHighCone = 53.697/4;

    public static final double kElevatorScoreHybrid = 0;
    public static final double kElevatorIntakeDoubleSubstation = 0;
    // public static final double kElevatorMotionAcceleration = 0;
    // public static final double kElevatorCruiseVelocity = 0;

    public static final double kUpElevatorP = 0.1;
    public static final double kUpElevatorI = 0;
    public static final double kUpElevatorD = 0.0005;

    public static final double kDownElevatorP = 0.01;
    public static final double kDownElevatorI = 0;
    public static final double kDownElevatorD = 0.0005;

    public static final int kUpElevatorSlot = 0;
    public static final int kDownElevatorSlot = 1;

    public static final double kBufferZone = 0;
    public static final double targetTime = 0.5;

    public static final double kManualSpeed = 1;
    public static final double elevatorThreshold = -14.0;
    public static final double maxTimeAllowed = 0.0;
  }

}
