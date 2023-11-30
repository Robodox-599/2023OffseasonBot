package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.CANSparkMaxUtil.Usage;
import frc.robot.Constants.SwerveConstants;

public class SwerveModule {
    private CANSparkMax m_DriveMotor;
    private CANSparkMax m_AngleMotor;
    private RelativeEncoder m_DriveEncoder;
    private RelativeEncoder m_AngleEncoder;

    private boolean m_DriveMotorInvert;
    private boolean m_AngleMotorInvert;
    
    private SparkMaxPIDController m_DrivePID;
    private SparkMaxPIDController m_AnglePID;
    // private boolean positive;
    private double m_LastAngle;
    
    // private final SimpleMotorFeedforward m_Feedforward = 
    //     new SimpleMotorFeedforward(SwerveConstants.driveKS,
    //                                 SwerveConstants.driveKV,
    //                                 SwerveConstants.driveKA);

    public SwerveModule(int driveMotorID, int angleMotorID,
                        boolean driveMotInvert, boolean angleMotInvert){
        m_DriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        
        m_DriveEncoder = m_DriveMotor.getEncoder();
        m_DrivePID = m_DriveMotor.getPIDController();
        m_DriveMotorInvert = driveMotInvert;
        // m_DriveEncoder.setPositionConversionFactor(SwerveConstants.driveConversionPositionFactor);
        // m_DriveEncoder.setVelocityConversionFactor(SwerveConstants.driveConversionVelocityFactor);
        configDriveMotor();

        m_AngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
        m_AngleEncoder = m_AngleMotor.getEncoder();
        // m_AngleEncoder.setPositionConversionFactor(SwerveConstants.angleConversionFactor);
        m_AngleMotorInvert = angleMotInvert;
        
        m_AnglePID = m_AngleMotor.getPIDController();
        m_AnglePID.setPositionPIDWrappingEnabled(true);
        m_AnglePID.setPositionPIDWrappingMaxInput(180);
        m_AnglePID.setPositionPIDWrappingMinInput(-180);
        // m_AnglePID.setPositionPIDWrappingMaxInput(360.0);
        // m_AnglePID.setPositionPIDWrappingMinInput(0.0);
        configAngleMotor();

        m_LastAngle = getState().angle.getDegrees();
    }

    public void configAngleMotor(){
        m_AngleMotor.restoreFactoryDefaults();
        Timer.delay(0.2);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_AngleMotor, Usage.kPositionOnly);
        m_AngleMotor.setSmartCurrentLimit(SwerveConstants.angleContinuousCurrentLimit);
        m_AngleMotor.setInverted(m_AngleMotorInvert);
        m_AngleMotor.setIdleMode(SwerveConstants.angleNeutralMode);
        
        m_AnglePID.setP(SwerveConstants.angleKP);
        m_AnglePID.setI(SwerveConstants.angleKI);
        m_AnglePID.setD(SwerveConstants.angleKD);
        m_AnglePID.setFF(SwerveConstants.angleKFF);
        
        m_AngleMotor.enableVoltageCompensation(SwerveConstants.voltageComp);
        Timer.delay(0.2);
        m_AngleMotor.burnFlash();
        Timer.delay(0.2);
        m_AngleEncoder.setPosition(0.0);
    }

    public void configDriveMotor(){
        m_DriveMotor.restoreFactoryDefaults();
        Timer.delay(0.2);
        CANSparkMaxUtil.setCANSparkMaxBusUsage(m_DriveMotor, Usage.kAll);
        m_DriveMotor.setSmartCurrentLimit(SwerveConstants.driveContinuousCurrentLimit);
        m_DriveMotor.setInverted(m_DriveMotorInvert);
        m_DriveMotor.setIdleMode(SwerveConstants.driveNeutralMode);
        
        m_DrivePID.setP(SwerveConstants.driveKP);
        m_DrivePID.setI(SwerveConstants.driveKI);
        m_DrivePID.setD(SwerveConstants.driveKD);
        m_DrivePID.setFF(SwerveConstants.driveKFF);

        m_DriveMotor.enableVoltageCompensation(SwerveConstants.voltageComp);

        Timer.delay(0.2);
        m_DriveMotor.burnFlash();
        Timer.delay(0.2);
        m_DriveEncoder.setPosition(0.0);
    }


    public double getAngle() { 
        SmartDashboard.putNumber("Current", m_AngleMotor.getOutputCurrent());
        double pos = neoToDegrees(m_AngleEncoder.getPosition());
        if(pos < -360.0){
            pos = pos % -360.0;
        }
        if(pos >= 360.0){
            pos = pos % 360.0;
        }
        if(pos <= -180.0 && pos > -360.0){
            pos += 360.0;
        }
        if(pos >= 180.0 && pos < 360.0){
            pos -= 360.0;
        }
        // m_AngleEncoder.setPosition(degreesToNEO(pos));
        return pos;
    }

    public double getLinearVelocity() {
        return m_DriveEncoder.getVelocity();
    }

    public double getAngularVelocity() {
        return m_AngleEncoder.getVelocity();
    }

    public double getAngularPos(){
        return neoToDegrees(m_AngleEncoder.getPosition());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getLinearVelocity(), Rotation2d.fromDegrees(getAngle()));
    }

    public SwerveModulePosition getPosition() {
        double r = m_DriveEncoder.getPosition();
        Rotation2d theta = Rotation2d.fromDegrees(getAngle());
        return new SwerveModulePosition(r, theta);
    }

    public SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle){
        double modReferenceAngle = MathUtil.angleModulus(currentAngle.getRadians()) * 180.0 / Math.PI;
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = desiredState.angle.getDegrees() - modReferenceAngle;
        if(delta >= 270.0){
            delta -= 360.0;
        }
        if(delta <= -270.0){
            delta += 360.0;
        }
        if(Math.abs(delta) > 90.0){
            targetSpeed *= -1.0;
            delta = delta > 0.0 ? delta - 180.0 : delta + 180.0;
        }
        double targetAngle = currentAngle.getDegrees() + delta;

        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    public void setDesiredState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, getState().angle);
        if (Math.abs(state.speedMetersPerSecond)<0.001){
            stop();
            return;
        }
        m_DriveMotor.set(state.speedMetersPerSecond / SwerveConstants.maxSpeed);
        double minSpeed = SwerveConstants.maxSpeed * 0.01;
        double angle = Math.abs(state.speedMetersPerSecond) <= minSpeed ? getAngle() : state.angle.getDegrees();
        // if(angle < 180.0 && angle > 179.0){
        //     angle = 180.0;
        // }
        // if(angle > -180.0 && angle < -179.0){
        //     angle = -180.0;
        // }
        // if(angle < 1.0 && angle > 0.0){
        //     angle = 0.0;
        // }
        // if(angle > -1.0 && angle < 0.0){
        //     angle = 0.0;
        // }
        m_AnglePID.setReference(angle, ControlType.kPosition);
            
        m_LastAngle = angle;
    }

    public void stop() {
        m_DriveMotor.set(0);
        m_AngleMotor.set(0);
    }

    public double degreesToNEO(double degrees){
        return degrees / SwerveConstants.angleConversionFactor;
        // return degrees;
    }

    public double neoToDegrees(double counts){
        // return counts * SwerveConstants.angleConversionFactor;
        return counts / SwerveConstants.angleGearRatio;
    }
}

