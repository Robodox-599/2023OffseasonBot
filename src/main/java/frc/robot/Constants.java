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
    public static final double kDeadband = 0.1;
    public static final int kLYAxis = 1;
    public static final int kLXAxis = 0;
    public static final int kRXAxis = 4;
    public static final int kY = 1;
  }
  
  public static class SwerveConstants {
    public static final double trackWidth = Units.inchesToMeters(19.976);
    public static final double wheelBase = Units.inchesToMeters(19.976);
    public static final double wheelDiameter = Units.inchesToMeters(3.75);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.75 / 1.0); // 6.75:1
    public static final double angleGearRatio = (150.0 / 7.0); // 150/7:1

    public static final Translation2d frontLeft = new Translation2d(Units.inchesToMeters(wheelBase / 2.0), Units.inchesToMeters(trackWidth / 2.0));
    public static final Translation2d frontRight = new Translation2d(Units.inchesToMeters(wheelBase / 2.0), -Units.inchesToMeters(trackWidth / 2.0));
    public static final Translation2d backLeft = new Translation2d(-Units.inchesToMeters(wheelBase / 2.0), Units.inchesToMeters(trackWidth / 2.0));
    public static final Translation2d backRight = new Translation2d(-Units.inchesToMeters(wheelBase / 2.0), -Units.inchesToMeters(trackWidth / 2.0));

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            frontLeft, frontRight, backLeft, backRight);

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 20;
    public static final int driveContinuousCurrentLimit = 80;

    /* Angle Motor PID Values */
    public static final double angleKP = 0.1;
    public static final double angleKI = 0.0;
    public static final double angleKD = 0.01;
    public static final double angleKFF = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.1;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.0;
    public static final double driveKV = 0.0;
    public static final double driveKA = 0.0;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / (driveGearRatio);
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / (angleGearRatio);
    
    /* Swerve Profiling Values */
    public static final double maxSpeed = 0.5; // meters per second
    public static final double maxAngularVelocity = 60.0 * 0.5;

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;
  }

  public static class FrontLeftModule {
    public static final int driveMotorID = 3;
    public static final int angleMotorID = 4;
    public static final boolean driveMotInvert = false;
    public static final boolean angleMotInvert = true;
  }

  public static class FrontRightModule {
    public static final int driveMotorID = 9;
    public static final int angleMotorID = 2;
    public static final boolean driveMotInvert = false;
    public static final boolean angleMotInvert = true;
  }

  public static class BackLeftModule {
    public static final int driveMotorID = 5;
    public static final int angleMotorID = 6;
    public static final boolean driveMotInvert = false;
    public static final boolean angleMotInvert = true;
  }

  public static class BackRightModule {
    public static final int driveMotorID = 7;
    public static final int angleMotorID = 8;
    public static final boolean driveMotInvert = false;
    public static final boolean angleMotInvert = true;
  }
}
