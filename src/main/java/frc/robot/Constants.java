package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.config.SwerveModuleConstants;

public final class Constants {

  public static final class Swerve {
    public static final double stickDeadband = 0.1;

    public static final int pigeonID = 12;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
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
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */

    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 3;
      public static final int angleMotorID = 4;
      public static final int canCoderID = 2;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 9;
      public static final int angleMotorID = 2;
      public static final int canCoderID = 5;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 5;
      public static final int angleMotorID = 6;
      public static final int canCoderID = 8;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 7;
      public static final int angleMotorID = 8;
      public static final int canCoderID = 11;
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0.0);
      public static final SwerveModuleConstants constants =
          new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class ElevatorConstants {
    public static final int kElevatorID = 10;
    public static final int kElevatorFollowerID = 11;
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
    public static final double elevatorThreshold = -14.0; //this prob needa be changed
    public static final double maxTimeAllowed = 0.0; //change this
  }

  
  public class WristConstants {

    public static final int motorID = 12;

    public static final double kWristI = 0.0;
    public static final double kWristP = 0.05;
    public static final double kWristD = 0.0005;
    public static final double start_pos = -0.5; // hard stop pos
    public static final double bufferZone = 0.0; // still needs defining
    public static final double stepperConstant = 0.1;
    public static final double targetTime = 0.1; // assuming this is a double value
    public static final double wristThreshold = -14.0;
    public static final double manualThreshold = 0.02;

    public static final double highConePos = 14.91; // 125.81
    public static final double highCubePos = 7.96; // 67.16
    public static final double midConePos = 9.96; // 84.03
    public static final double midCubePos = 9.96;
    public static final double substationPos = 10.9048; // fix
    public static final double groundIntake = 16.809; // fix

  }

  public static class IntakeConstants{
    public static enum intakeState{
      noGamepiece,
      hasCone,
      hasCube,
      coneIntaking, 
      coneOuttaking, 
      cubeIntaking, 
      cubeOuttaking
    };

    public static final double normalOutputCurrent = 30.0;
    public static final int intakeMotorID = 13;
    public static final int intakeMotorCurrentLimit = 50;
    public static final double currentSpikeTimeSeconds = 0.15;
  }

  public static class LEDConstants{
    public static enum LEDState{
      standby, yellow, purple, error, off
    };    
    public static final int candleID = 20;
    public static final int intakeFlashTimeSeconds = 1;
  }

  public static final class ControllerConstants{
    public static final double deadband = 0.1;
    public static final double triggerActivate = 0.8;
    
    public static final int driveRadID = 0;
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
}
