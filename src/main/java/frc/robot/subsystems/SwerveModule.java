package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.kinematics.SwerveModuleState;
// import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.lib.CANSparkMaxUtil;
import frc.lib.CANSparkMaxUtil.Usage;

public class SwerveModule {
    private int moduleNumber;
    private Rotation2d lastAngle;
    // private Rotation2d angleOffset
    private CANSparkMax driveMotor;
    private CANSparkMax turnMotor;
    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    private final SparkMaxPIDController turnPID;
    private final SparkMaxPIDController drivePID;
    
    private final SimpleMotorFeedforward feedForward = 
        new SimpleMotorFeedforward(moduleNumber, moduleNumber, moduleNumber);


    public SwerveModule(int driveMotorID, int turnMotorID, boolean driveMotorReverse, boolean turnMotorReverse){

        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnMotorID, MotorType.kBrushless);
        driveMotor.setInverted(driveMotorReverse);
        turnMotor.setInverted(turnMotorReverse);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        turnPID = turnMotor.getPIDController();
        turnPID.getPositionPIDWrappingEnabled();
        turnPID.setPositionPIDWrappingMaxInput(180);
        turnPID.setPositionPIDWrappingMinInput(-180);

        drivePID = driveMotor.getPIDController();
    }

    public void configAngleMotor(){
        turnMotor.restoreFactoryDefaults();
        Timer.delay(1.0);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(turnMotor, Usage.kPositionOnly);
        turnMotor.setSmartCurrentLimit(4);
        turnMotor.setInverted(false);
        turnMotor.setIdleMode(IdleMode.kCoast);
        
        turnPID.setP(0.0);
        turnPID.setI(0.0);
        turnPID.setD(0.0);
        turnPID.setFF(0.0);
        
        turnMotor.enableVoltageCompensation(12.0);
        Timer.delay(1.0);
        turnMotor.burnFlash();
        Timer.delay(1.0);
    }

    public void configDriveMotor(){
        driveMotor.restoreFactoryDefaults();
        Timer.delay(1.0);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
        driveMotor.setSmartCurrentLimit(10);
        driveMotor.setInverted(false);
        driveMotor.setIdleMode(IdleMode.kBrake);
        
        drivePID.setP(0.0);
        drivePID.setI(0.0);
        drivePID.setD(0.0);
        drivePID.setFF(0.0);

        driveMotor.enableVoltageCompensation(12.0);

        Timer.delay(1.0);
        driveMotor.burnFlash();
        Timer.delay(1.0);
        driveEncoder.setPosition(0.0);
    }


    public double getTurningPosition() {
        return turnEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turnEncoder.getVelocity();
    }


    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond);
        // turnMotor.set(turnPID.calculate(getTurningPosition(), state.angle.getRadians()));
        
    }

    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}