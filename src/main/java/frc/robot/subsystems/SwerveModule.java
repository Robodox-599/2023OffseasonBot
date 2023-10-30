package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.config.SwerveModuleConstants;
import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

  Mechanism2d moduleVisualizer = new Mechanism2d(Units.inchesToMeters(6), Units.inchesToMeters(6));
  MechanismRoot2d moduleRoot;
  MechanismLigament2d moduleLigament;

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    /* Angle Encoder Config */
    angleEncoder = new CANCoder(moduleConstants.cancoderID, "BarryDriveCANivore");
    configAngleEncoder();


    lastAngle = getState().angle;

    moduleRoot =
        moduleVisualizer.getRoot(
            "ModuleCenter_" + moduleNumber,
            Units.inchesToMeters(3),
            Units.inchesToMeters(3));
    moduleLigament =
        moduleRoot.append(
            new MechanismLigament2d(
                "ModuleDirection_" + moduleNumber,
                (getState().speedMetersPerSecond / Constants.Swerve.maxSpeed) * .75 + .25,
                getState().angle.getDegrees()));
    SmartDashboard.putData("Module" + moduleNumber, moduleVisualizer);
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which
    // REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void resetToAbsolute() {
    System.out.println(" " + moduleNumber + " Angle Encoder resetToAbsolute()");
    System.out.println(" " + moduleNumber + " getCanCoder().getDegrees() " + getCanCoder().getDegrees());
    System.out.println(" " + moduleNumber + " angleOffset" + angleOffset.getDegrees());
    System.out.println(" " + moduleNumber + " integratedAngleEncoder original Angle " + integratedAngleEncoder.getPosition());
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    System.out.println(" " + moduleNumber + " integratedAngleEncoder new absolutePosition " + absolutePosition);
    integratedAngleEncoder.setPosition(absolutePosition);
    Timer.delay(0.2);
    System.out.println(" " + moduleNumber + " integratedAngleEncoder new Angle " + integratedAngleEncoder.getPosition());
  }

  private void configAngleEncoder() {
    System.out.println("Starting " + moduleNumber + " Angle Encoder Config");
    Timer.delay(0.2);
    angleEncoder.configFactoryDefault();
    Timer.delay(0.2);
    System.out.println(" " + moduleNumber + " Angle Encoder Absolute Angle" + angleEncoder.getAbsolutePosition());
    System.out.println(" " + moduleNumber + " Angle Encoder Relative Angle" + angleEncoder.getPosition());
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
    resetToAbsolute();
  }

  private void configAngleMotor() {
    System.out.println("Starting " + moduleNumber + " Angle Motor Config");
    Timer.delay(0.2);
    angleMotor.restoreFactoryDefaults();
    Timer.delay(0.2);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.Swerve.angleConversionFactor);
    angleController.setFeedbackDevice(integratedAngleEncoder);
    angleController.setPositionPIDWrappingEnabled(true);
    angleController.setPositionPIDWrappingMinInput(0.0);
    angleController.setPositionPIDWrappingMaxInput(360.0);
    angleController.setP(Constants.Swerve.angleKP);
    angleController.setI(Constants.Swerve.angleKI);
    angleController.setD(Constants.Swerve.angleKD);
    angleController.setFF(Constants.Swerve.angleKFF);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp); 
    
    Timer.delay(0.2);
    angleMotor.burnFlash();
    Timer.delay(0.2);
    System.out.println("" + moduleNumber + " Angle Initial Angle: " + integratedAngleEncoder.getPosition());
  }

  private void configDriveMotor() {
    System.out.println("Starting " + moduleNumber + " Drive Motor Config");
    Timer.delay(0.2);
    driveMotor.restoreFactoryDefaults();
    Timer.delay(0.2);
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.angleKP);
    driveController.setI(Constants.Swerve.angleKI);
    driveController.setD(Constants.Swerve.angleKD);
    driveController.setFF(Constants.Swerve.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    
    Timer.delay(0.2);
    driveMotor.burnFlash();
    Timer.delay(0.2);
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(driveEncoder.getPosition(), getAngle());
  }

  public void periodic() {
    moduleLigament.setLength(
        ((getState().speedMetersPerSecond / Constants.Swerve.maxSpeed) * .75) + .25);
    moduleLigament.setAngle(getState().angle.getDegrees());
  }
}
