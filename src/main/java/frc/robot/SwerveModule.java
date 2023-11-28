package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CANSparkMaxUtil.Usage;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private CANSparkMax m_DriveMotor;
    private CANSparkMax m_AngleMotor;
    private RelativeEncoder m_DriveEncoder;
    private RelativeEncoder m_AngleEncoder;
    
    private SparkMaxPIDController m_DrivePID;
    private SparkMaxPIDController m_AnglePID;
    // private boolean positive;
    private double m_LastAngle;
    
    // private final SimpleMotorFeedforward m_Feedforward = 
    //     new SimpleMotorFeedforward(SwerveConstants.driveKS,
    //                                 SwerveConstants.driveKV,
    //                                 SwerveConstants.driveKA);

    public SwerveModule(int driveMotorID, int angleMotorID,
                        boolean driveEncInvert, boolean angleEncInvert){
        m_DriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        m_DriveMotor.setInverted(driveEncInvert);
        m_DriveEncoder = m_DriveMotor.getEncoder();
        m_DrivePID = m_DriveMotor.getPIDController();
        configDriveMotor();

        m_AngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        m_AngleEncoder = m_AngleMotor.getEncoder();
        m_AngleMotor.setInverted(angleEncInvert);
        
        m_AnglePID = m_AngleMotor.getPIDController();
        m_AnglePID.getPositionPIDWrappingEnabled();
        m_AnglePID.setPositionPIDWrappingMaxInput(180);
        m_AnglePID.setPositionPIDWrappingMinInput(-180);
        configAngleMotor();

        m_LastAngle = getPosition().angle.getDegrees();
    }

    public void configAngleMotor(){
        m_AngleMotor.restoreFactoryDefaults();
        Timer.delay(0.5);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_AngleMotor, Usage.kPositionOnly);
        m_AngleMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
        m_AngleMotor.setInverted(false);
        m_AngleMotor.setIdleMode(SwerveConstants.angleNeutralMode);
        
        m_AnglePID.setP(SwerveConstants.angleKP);
        m_AnglePID.setI(SwerveConstants.angleKI);
        m_AnglePID.setD(SwerveConstants.angleKD);
        m_AnglePID.setFF(SwerveConstants.angleKFF);
        
        m_AngleMotor.enableVoltageCompensation(SwerveConstants.voltageComp);
        Timer.delay(0.5);
        m_AngleMotor.burnFlash();
        Timer.delay(0.5);
        m_AngleEncoder.setPosition(0.0);
    }

    public void configDriveMotor(){
        m_DriveMotor.restoreFactoryDefaults();
        Timer.delay(0.5);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_DriveMotor, Usage.kAll);
        m_DriveMotor.setSmartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
        m_DriveMotor.setInverted(false);
        m_DriveMotor.setIdleMode(SwerveConstants.driveNeutralMode);
        
        m_DrivePID.setP(SwerveConstants.driveKP);
        m_DrivePID.setI(SwerveConstants.driveKI);
        m_DrivePID.setD(SwerveConstants.driveKD);
        m_DrivePID.setFF(SwerveConstants.driveKFF);

        m_DriveMotor.enableVoltageCompensation(SwerveConstants.voltageComp);

        Timer.delay(0.5);
        m_DriveMotor.burnFlash();
        Timer.delay(0.5);
        m_DriveEncoder.setPosition(0.0);
    }


    public double getAngle() { 
        SmartDashboard.putNumber("Current", m_AngleMotor.getOutputCurrent());
        return getPosition().angle.getDegrees();
    }

    public double getLinearVelocity() {
        return m_DriveEncoder.getVelocity();
    }

    public double getAngularVelocity() {
        return m_AngleEncoder.getVelocity();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getLinearVelocity(), new Rotation2d(getAngle()));
    }

    public SwerveModulePosition getPosition() {
        double r = (m_DriveEncoder.getPosition() / 42.0) * SwerveConstants.driveConversionPositionFactor;
        Rotation2d theta = Rotation2d.fromDegrees(m_AngleEncoder.getPosition() / 42.0 * SwerveConstants.angleConversionFactor);
        return new SwerveModulePosition(r, theta);
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        m_DriveMotor.set(state.speedMetersPerSecond / SwerveConstants.maxSpeed);
        double minSpeed = SwerveConstants.maxSpeed * 0.01;
        double angle = Math.abs(state.speedMetersPerSecond) <= minSpeed ? m_LastAngle : state.angle.getDegrees();
        // m_AnglePID.setReference(angle, ControlType.kPosition);
        
        // if(getAngle()>0){
        //      positive = true;
        // } else{
        //      positive = false;
        // }
        // if(Math.abs(Math.signum(getAngle()) * (Math.abs(getAngle()) % 180) - angle) <= 10){
            
        // } else {
            
        // }

        if(Math.abs(Math.signum(getAngle()) * (Math.abs(getAngle()) % 180) - angle) > 10){
            m_AngleMotor.set(0.1 * Math.signum(getAngle()));
        } else {
            m_AngleMotor.set(0.0);
        }
        // SmartDashboard.putNumber("Reference", state.angle.getDegrees());
        m_LastAngle = angle;
    }
    public void stop() {
        m_DriveMotor.set(0);
        m_AngleMotor.set(0);
    }
}

