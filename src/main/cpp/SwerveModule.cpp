#include "SwerveModule.h"
#include "frc/smartdashboard/SmartDashboard.h"

SwerveModule::SwerveModule(const double Module[] ):
                                                //  m_DriveMotor{ (int)Module[0], "DriveCANivore"},
                                                //  m_AngleMotor{ (int)Module[1], "DriveCANivore" },
                                                m_DriveMotor{(int)Module[0], rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                                m_AngleMotor{(int)Module[1], rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                                m_AngleCANcoder{ (int)Module[2], "DriveCANivore" },
                                                m_AngleOffset{ Module[3] },
                                                m_DrivePID{m_DriveMotor.GetPIDController()},
                                                m_AnglePID{m_AngleMotor.GetPIDController()},
                                                m_DriveRelEncoder{m_DriveMotor.GetEncoder()},
                                                m_AngleRelEncoder{m_AngleMotor.GetEncoder()}, 
                                                m_Feedforward{SwerveConstants::DriveKS, SwerveConstants::DriveKV, SwerveConstants::DriveKA}
{
    m_SwerveCanCoderConfig.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
    m_SwerveCanCoderConfig.sensorDirection = SwerveConstants::CanCoderInvert;
    m_SwerveCanCoderConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
    m_SwerveCanCoderConfig.sensorTimeBase = ctre::phoenix::sensors::SensorTimeBase::PerSecond;
    
    //Config Angle Encoder
    m_AngleCANcoder.ConfigFactoryDefault();
    m_AngleCANcoder.ConfigAllSettings(m_SwerveCanCoderConfig);
    
    //Config Angle Motor
    m_AngleMotor.RestoreFactoryDefaults();
    m_AngleMotor.SetSmartCurrentLimit(SwerveConstants::AngleContinuousCurrentLimit);
    m_AngleMotor.SetInverted(SwerveConstants::AngleMotorInvert);
    m_AngleMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // m_AngleRelEncoder.SetPositionConversionFactor(360.0 / SwerveConstants::AngleGearRatio);

    m_AnglePID.SetP(SwerveConstants::AngleKP, 0);
    m_AnglePID.SetI(SwerveConstants::AngleKI, 0);
    m_AnglePID.SetD(SwerveConstants::AngleKD, 0);
    m_AnglePID.SetFF(SwerveConstants::AngleKF, 0);

    m_AngleMotor.EnableVoltageCompensation(SwerveConstants::kNominalDouble);
    m_AngleMotor.BurnFlash();

    ResetToAbsolute();

    // Config Drive Motor
    m_DriveMotor.RestoreFactoryDefaults();
    m_DriveMotor.SetSmartCurrentLimit(SwerveConstants::DriveContinuousCurrentLimit);
    m_DriveMotor.SetInverted(SwerveConstants::DriveMotorInvert);
    m_DriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // double pos_conv_factor = SwerveConstants::WheelCircumference / SwerveConstants::DriveGearRatio;

    // m_DriveRelEncoder.SetPositionConversionFactor(pos_conv_factor);
    // m_DriveRelEncoder.SetVelocityConversionFactor(pos_conv_factor / 60.0);

    m_DrivePID.SetP(SwerveConstants::DriveKP, 0);
    m_DrivePID.SetI(SwerveConstants::DriveKI, 0);
    m_DrivePID.SetD(SwerveConstants::DriveKD, 0);
    m_DrivePID.SetFF(SwerveConstants::DriveKF, 0);

    m_DriveMotor.EnableVoltageCompensation(SwerveConstants::kNominalDouble);
    m_DriveMotor.BurnFlash();
    
    m_DriveRelEncoder.SetPosition(0.0);

    // m_AngleMotor.ConfigFactoryDefault();
    // m_AngleMotor.ConfigAllSettings(m_Settings.SwerveAngleFXConfig);
    // m_AngleMotor.SetInverted(SwerveConstants::AngleMotorInvert);
    // m_AngleMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Coast);
    // m_AngleMotor.SetSelectedSensorPosition(( DegreesToNEO(GetCANCoder().Degrees() - m_AngleOffset) ));

    //Config Drive Motor
    // m_DriveMotor.ConfigFactoryDefault();
    // m_DriveMotor.ConfigAllSettings(m_Settings.SwerveDriveFXConfig);
    // m_DriveMotor.SetInverted(SwerveConstants::DriveMotorInvert);
    // m_DriveMotor.SetNeutralMode(ctre::phoenix::motorcontrol::NeutralMode::Brake);
    // m_DriveMotor.SetSelectedSensorPosition(0);
    // m_DriveMotor.EnableVoltageCompensation(true);

    // m_LastAngle = GetState().angle.Degrees();
    // m_DriveMotor.RestoreFactoryDefaults();
    // m_AngleMotor.RestoreFactoryDefaults();
    
    // rev::AbsoluteEncoder m_AngleEncoder = m_AngleMotor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle);
    // m_AngleEncoder.SetInverted(true);
    
    // m_AngleMotor.SetFeedbackDevice(m_AngleEncoder);

    // rev::CANPIDController m_DrivePIDController = m_DriveMotor.GetPIDController();
    // rev::CANPIDController m_AnglePIDController = m_AngleMotor.GetPIDController();

    // m_AnglePIDController.SetFeedbackDevice(m_AngleEncoder);
    // m_AnglePIDController.SetP(SwerveConstants::AngleKP);
    // m_AnglePIDController.SetI(SwerveConstants::AngleKI);
    // m_AnglePIDController.SetD(SwerveConstants::AngleKD);
    // // m_AnglePIDController.SetIZone(k);
    // // m_AnglePIDController.SetFF(k);
    // m_AnglePIDController.SetOutputRange(0.0,1.0);
    // m_AnglePIDController.EnableContinuousInput(-1.0, 1.0);
    // m_AnglePIDController.SetTolerance(0.02);

    // m_DrivePIDController.SetP(SwerveConstants::DriveKP);
    // m_DrivePIDController.SetI(SwerveConstants::DriverKI);
    // m_DrivePIDController.SetD(SwerveConstants::DriveKD);
    // // m_DrivePIDController.SetIZone(k);
    // // m_DrivePIDController.SetFF(k);
    // m_DrivePIDController.SetOutputRange(0.0,1.0);
    // m_DrivePIDController.EnableContinuousInput(-1.0, 1.0);
    // m_DrivePIDController.SetTolerance(0.02);
    



    // m_AngleEncoder.SetInverted(true);
    // m_AngleMotor.SetFeedbackDevice(m_AngleEncoder);
    
}


double SwerveModule::getTurnCounts(){
    return m_AngleRelEncoder.GetPosition();
}


void SwerveModule::SetDesiredState(frc::SwerveModuleState& DesiredState, bool IsOpenLoop){
    DesiredState = Optimize(DesiredState, GetState().angle);
    if(IsOpenLoop){
        double PercentOutput = DesiredState.speed / SwerveConstants::MaxSpeed;
        m_DriveMotor.Set(PercentOutput);
    }else{  
        double VoltageFeedForward = m_Feedforward.Calculate(DesiredState.speed)/SwerveConstants::kNominal;
        m_DrivePID.SetReference(
            DesiredState.speed.value(), 
            rev::ControlType::kVelocity,
            0,
            VoltageFeedForward,
            rev::CANPIDController::ArbFFUnits::kVoltage
        );
        // m_DrivePID.SetReference(
        //     DesiredState.speed.value(), 
        //     rev::ControlType::kVelocity, 
        //     0, 
        //     VoltageFeedForward, 
        //     rev::CANPIDController::ArbFFUnits::kVoltage
        // );
        // m_DriveMotor.Set(PercentOutput);
        // m_DriveMotor.SetVoltage(VoltageFeedForward);
    }
    units::meters_per_second_t minSpeed = (SwerveConstants::MaxSpeed * 0.01);

    units::degree_t Angle = (units::math::abs(DesiredState.speed) <= minSpeed  ) ? m_LastAngle: DesiredState.angle.Degrees();
    m_AngleRelEncoder.SetPosition(DegreesToNEO(Angle));
     // m_AngleMotor.Set( ctre::phoenix::motorcontrol::ControlMode::Position,  );
    m_LastAngle = Angle;

    /*

    DesiredState = Optimize(DesiredState, GetState().angle);
    units::meters_per_second_t minSpeed = (SwerveConstants::MaxSpeed * 0.01);

    if(IsOpenLoop){
        double PercentOutput = DesiredState.speed / SwerveConstants::MaxSpeed;
        m_DriveMotor.Set(PercentOutput);
    }else{
        double Velocity = MPSToNEO(DesiredState.speed);  
        double VoltageFeedForward = m_Feedforward.Calculate(DesiredState.speed)/SwerveConstants::kNominal;
        if(Velocity > minSpeed.value()){
            m_DriveMotor.Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, Velocity, ctre::phoenix::motorcontrol::DemandType_ArbitraryFeedForward, VoltageFeedForward);
        }else{
            m_DriveMotor.Set(0.0);
        }
    }

    units::degree_t Angle = (units::math::abs(DesiredState.speed) <= minSpeed  ) ? m_LastAngle: DesiredState.angle.Degrees();


    m_AngleMotor.Set( ctre::phoenix::motorcontrol::ControlMode::Position, DegreesToNEO(Angle) );
    m_LastAngle = Angle;
    */
}

void SwerveModule::SetDesiredAngle(frc::Rotation2d Angle){
    frc::SwerveModuleState TempState {0_mps, Angle};
    TempState= Optimize(TempState, GetState().angle);
    // m_AngleMotor.Set( ctre::phoenix::motorcontrol::ControlMode::Position,  );
    m_AngleRelEncoder.SetPosition(DegreesToNEO(Angle.Degrees()));
    m_LastAngle = Angle.Degrees();
}

units::degree_t SwerveModule::getLastAngle(){
    return m_LastAngle;
}

/* This custom optimize method is created because Wpilib assumes the controller is continuous, which the CTRE Talons are not. */
frc::SwerveModuleState SwerveModule::Optimize(frc::SwerveModuleState DesiredState, frc::Rotation2d CurrentAngle){

    units::degree_t ModReferenceAngle { frc::AngleModulus( CurrentAngle.Radians() )  };
    // frc::SmartDashboard::SmartDashboard::PutNumber("Current Angle", ModReferenceAngle.value());
    // frc::SmartDashboard::SmartDashboard::PutNumber("Desired Angle(Continuous)", DesiredState.angle.Degrees().value());

    units::meters_per_second_t TargetSpeed = DesiredState.speed;
    units::degree_t Delta = DesiredState.angle.Degrees() - ModReferenceAngle;
    if(Delta >= 270_deg){
        Delta -= 360_deg;
    }else if(Delta <= -270_deg){
        Delta += 360_deg;
    }
    if( units::math::abs(Delta) > 90_deg){
        TargetSpeed = - TargetSpeed;
        Delta = Delta > 0_deg ? (Delta -= 180_deg) : ( Delta += 180_deg);
    }
    units::degree_t TargetAngle = CurrentAngle.Degrees() + Delta;
    // frc::SmartDashboard::SmartDashboard::PutNumber("Desired Angle(Discontinuous)", DesiredState.angle.Degrees().value());

    return  {TargetSpeed, TargetAngle};
}

void SwerveModule::ResetToAbsolute(){
    m_AngleRelEncoder.SetPosition(( DegreesToNEO(GetCANCoder().Degrees() - m_AngleOffset) ));
}

frc::Rotation2d SwerveModule::GetCANCoder(){
    return frc::Rotation2d( units::degree_t( m_AngleCANcoder.GetAbsolutePosition() ) );
}

frc::SwerveModulePosition SwerveModule::GetPosition(){
    units::meter_t Distance{ NEOToMeters(m_DriveRelEncoder.GetPosition())};
    frc::Rotation2d Angle{NEOToDegrees( m_AngleRelEncoder.GetPosition()) };
    return {Distance, Angle};

}

units::meter_t SwerveModule::NEOToMeters(double Counts){
    return units::meter_t{ (Counts * SwerveConstants::WheelCircumference) / ( 42.0 * SwerveConstants::DriveGearRatio)};
}

frc::SwerveModuleState SwerveModule::GetState(){
    units::meters_per_second_t Velocity{ RPMToMPS(m_DriveRelEncoder.GetVelocity()) };
    frc::Rotation2d Angle{NEOToDegrees( m_AngleRelEncoder.GetPosition()) };
    return {Velocity, Angle};
}

units::degree_t SwerveModule::NEOToDegrees(double Counts){
    return units::degree_t(Counts * ( 360.0 / (SwerveConstants::AngleGearRatio * 42.0)));
}

double SwerveModule::DegreesToNEO(units::degree_t Degrees){
    return Degrees.value() / (360.0 / (SwerveConstants::AngleGearRatio * 42.0));
}

// double SwerveModule::NEOToRPM(double VelocityCounts){
//     double MotorRPM = VelocityCounts * ( 600.0 / 2048.0);
//     double MechRPM = MotorRPM / SwerveConstants::DriveGearRatio;
//     return MechRPM;
// }

// double SwerveModule::RPMToNEO(double RPM){
//     double MotorRPM = RPM * SwerveConstants::DriveGearRatio;
//     double SensorCounts = MotorRPM * (2048.0 / 600.0);
//     return SensorCounts;
// }

units::meters_per_second_t SwerveModule::RPMToMPS(double VelocityCounts){
    double WheelRPM = VelocityCounts;
    units::meters_per_second_t WheelMPS = (WheelRPM * SwerveConstants::WheelCircumference) / 60_s;
    return WheelMPS;
}

// double SwerveModule::MPSToNEO(units::meters_per_second_t Velocity){
//     double WheelRPM = ( Velocity.value() * 60 ) / SwerveConstants::WheelCircumference.value();
//     double WheelVelocity = RPMToNEO(WheelRPM);
//     return WheelVelocity;
// }
