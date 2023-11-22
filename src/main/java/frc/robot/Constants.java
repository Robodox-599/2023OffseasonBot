// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.01;
    public static final int kDriverPort = 0;

    public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;
  }
  


  public static class DriveConstants{
    public static final int kFrontLeftDriveMotorPort = 3;
    public static final int kFrontLeftTurnMotorPort = 4;
    public static final boolean kFrontLeftDriveEncoderReversed = false;
    public static final boolean kFrontLeftTurnEncoderReversed = false;

    public static final int kFrontRightDriveMotorPort = 9;
    public static final int kFrontRightTurnMotorPort = 2;
    public static final boolean kFrontRightDriveEncoderReversed = false;
    public static final boolean kFrontRightTurnEncoderReversed = false;

    public static final int kBackLeftDriveMotorPort = 5;
    public static final int kBackLeftTurnMotorPort = 6;
    public static final boolean kBackLeftDriveEncoderReversed = false;
    public static final boolean kBackLeftTurnEncoderReversed = false;

    public static final int kBackRightDriveMotorPort = 7;
    public static final int kBackRightTurnMotorPort = 8;
    public static final boolean kBackRightDriveEncoderReversed = false;
    public static final boolean kBackRightTurnEncoderReversed = false;

    public static final double trackWidth = Units.inchesToMeters(19.976);
    public static final double wheelBase = Units.inchesToMeters(19.976);
    public static final double wheelDiameter = Units.inchesToMeters(3.75);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (150.0 / 7.0); // 150/7:1

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.0;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    // public static final double driveKS = 0.667;
    // public static final double driveKV = 2.44;
    // public static final double driveKA = 0.27;
    public static final double driveKS = 0.0;
    public static final double driveKV = 0.0;
    public static final double driveKA = 0.0;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.5; // meters per second
    public static final double maxAngularVelocity = 11.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;
    public static final IdleMode driveNeutralMode = IdleMode.kCoast;

    /* Motor Inverts */
    // public static final boolean driveInvert = false;
    // public static final boolean angleInvert = false;
  }
}
