#include "SwerveModule.h"
#include "frc/smartdashboard/SmartDashboard.h"

SwerveModule::SwerveModule(const double Module[] ):
                                                m_DriveMotor{(int)Module[0], rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                                m_AngleMotor{(int)Module[1], rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                                m_AngleCANcoder{ (int)Module[2], "BarryDriveCANivore" },
                                                m_AngleOffset{ Module[3] },
                                                m_DrivePID{m_DriveMotor.GetPIDController()},
                                                m_AnglePID{m_AngleMotor.GetPIDController()},
                                                m_DriveRelEncoder{m_DriveMotor.GetEncoder()},
                                                m_AngleRelEncoder{m_AngleMotor.GetEncoder()}, 
                                                // m_Feedforward{SwerveConstants::DriveKS, SwerveConstants::DriveKV, SwerveConstants::DriveKA}
                                                m_Feedforward{SwerveConstants::DriveKS, SwerveConstants::DriveKV, SwerveConstants::DriveKA} {
    m_SwerveCanCoderConfig.absoluteSensorRange = ctre::phoenix::sensors::AbsoluteSensorRange::Unsigned_0_to_360;
    m_SwerveCanCoderConfig.sensorDirection = SwerveConstants::CanCoderInvert;
    m_SwerveCanCoderConfig.initializationStrategy = ctre::phoenix::sensors::SensorInitializationStrategy::BootToAbsolutePosition;
    m_SwerveCanCoderConfig.sensorTimeBase = ctre::phoenix::sensors::SensorTimeBase::PerSecond;
    
    //Config Angle Encoder
    m_AngleCANcoder.ConfigFactoryDefault();
    SetCCUsage(SwerveConstants::CCUsage::kMinimalCC);
    SetCANCoderBusUsage();
    m_AngleCANcoder.ConfigAllSettings(m_SwerveCanCoderConfig);
    
    //Config Angle Motor
    m_AngleMotor.RestoreFactoryDefaults();
    SetUsage(SwerveConstants::Usage::kPositionOnly);
    SetAngleCANSparkMaxBusUsage();
    m_AngleMotor.SetSmartCurrentLimit(SwerveConstants::AngleContinuousCurrentLimit);
    m_AngleMotor.SetInverted(SwerveConstants::AngleMotorInvert);
    m_AngleMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // m_AnglePID.SetP(SwerveConstants::AngleKP, 0);
    // m_AnglePID.SetI(SwerveConstants::AngleKI, 0);
    // m_AnglePID.SetD(SwerveConstants::AngleKD, 0);
    // m_AnglePID.SetFF(SwerveConstants::AngleKF, 0);

    m_AnglePID.SetP(0.05, 0);
    m_AnglePID.SetI(0, 0);
    m_AnglePID.SetD(0, 0);
    m_AnglePID.SetFF(SwerveConstants::AngleKF, 0);

    m_AngleMotor.EnableVoltageCompensation(SwerveConstants::kNominalDouble);
    m_AngleMotor.BurnFlash();

    ResetToAbsolute();

    // Config Drive Motor
    m_DriveMotor.RestoreFactoryDefaults();
    SetUsage(SwerveConstants::Usage::kAll);
    SetDriveCANSparkMaxBusUsage();
    m_DriveMotor.SetSmartCurrentLimit(SwerveConstants::DriveContinuousCurrentLimit);
    m_DriveMotor.SetInverted(SwerveConstants::DriveMotorInvert);
    m_DriveMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kCoast);

    // double pos_conv_factor = SwerveConstants::WheelCircumference / SwerveConstants::DriveGearRatio;

    // m_DriveRelEncoder.SetPositionConversionFactor(pos_conv_factor);
    // m_DriveRelEncoder.SetVelocityConversionFactor(pos_conv_factor / 60.0);

    // m_DrivePID.SetP(SwerveConstants::DriveKP, 0);
    // m_DrivePID.SetI(SwerveConstants::DriveKI, 0);
    // m_DrivePID.SetD(SwerveConstants::DriveKD, 0);
    // m_DrivePID.SetFF(SwerveConstants::DriveKF, 0);

    m_DrivePID.SetP(0, 0);
    m_DrivePID.SetI(0, 0);
    m_DrivePID.SetD(0, 0);
    m_DrivePID.SetFF(0, 0);

    m_DriveMotor.EnableVoltageCompensation(SwerveConstants::kNominalDouble);
    m_DriveMotor.BurnFlash();
    
    m_DriveRelEncoder.SetPosition(0.0);
    
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
    }
    units::meters_per_second_t minSpeed = (SwerveConstants::MaxSpeed * 0.01);

    units::degree_t Angle = (fabs(DesiredState.speed.value()) <= minSpeed.value()  ) ? m_LastAngle: DesiredState.angle.Degrees();
    m_AnglePID.SetReference(DegreesToNEO(Angle), rev::ControlType::kPosition, 0);
     // m_AngleMotor.Set( ctre::phoenix::motorcontrol::ControlMode::Position,  );
    m_LastAngle = Angle;

    switch (m_AngleMotor.GetDeviceId())
    {
    case FrontLeftModule::AngleMotorID:
        frc::SmartDashboard::SmartDashboard::PutNumber("FL Last Angle", m_LastAngle.value());
        frc::SmartDashboard::SmartDashboard::PutNumber("FL CANCoder Angle", m_AngleCANcoder.GetPosition());
        break;
    
    case FrontRightModule::AngleMotorID:
        frc::SmartDashboard::SmartDashboard::PutNumber("FR Last Angle", m_LastAngle.value());
        frc::SmartDashboard::SmartDashboard::PutNumber("FR CANCoder Angle", m_AngleCANcoder.GetPosition());
        break;

    case BackLeftModule::AngleMotorID:
        frc::SmartDashboard::SmartDashboard::PutNumber("BL Last Angle", m_LastAngle.value());
        frc::SmartDashboard::SmartDashboard::PutNumber("BL CANCoder Angle", m_AngleCANcoder.GetPosition());
        break;

    case BackRightModule::AngleMotorID:
        frc::SmartDashboard::SmartDashboard::PutNumber("BR Last Angle", m_LastAngle.value());
        frc::SmartDashboard::SmartDashboard::PutNumber("BR CANCoder Angle", m_AngleCANcoder.GetPosition());
        break;
    
    default:
        frc::SmartDashboard::SmartDashboard::PutBoolean("Motor Not Found", false);
        break;
    }

    
}

void SwerveModule::SetDesiredAngle(frc::Rotation2d Angle){
    frc::SwerveModuleState TempState {0_mps, Angle};
    TempState = Optimize(TempState, GetState().angle);
    // m_AngleMotor.Set( ctre::phoenix::motorcontrol::ControlMode::Position,  );
    m_AnglePID.SetReference(DegreesToNEO(Angle.Degrees()), rev::ControlType::kPosition, 0);
    m_LastAngle = Angle.Degrees();
}

units::degree_t SwerveModule::getLastAngle(){
    return m_LastAngle;
}

/* This custom optimize method is created because Wpilib assumes the controller is continuous, which the CTRE Talons are not. */
frc::SwerveModuleState SwerveModule::Optimize(frc::SwerveModuleState DesiredState, frc::Rotation2d CurrentAngle){

    units::degree_t ModReferenceAngle { frc::AngleModulus( CurrentAngle.Radians() )  };
    frc::SmartDashboard::SmartDashboard::PutNumber("Current Angle", ModReferenceAngle.value());
    frc::SmartDashboard::SmartDashboard::PutNumber("Desired Angle(Continuous)", DesiredState.angle.Degrees().value());

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
    frc::SmartDashboard::SmartDashboard::PutNumber("Desired Angle(Discontinuous)", DesiredState.angle.Degrees().value());

    return  {TargetSpeed, TargetAngle};
}

void SwerveModule::ResetToAbsolute(){
    m_AngleRelEncoder.SetPosition(( DegreesToNEO(GetCANCoder().Degrees() - m_AngleOffset) ));
}

frc::Rotation2d SwerveModule::GetCANCoder(){
    double Pos = m_AngleCANcoder.GetPosition();
    frc::SmartDashboard::SmartDashboard::PutNumber("Error Code", m_AngleCANcoder.GetLastError());
    return frc::Rotation2d( units::degree_t( Pos) );
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
    return units::degree_t{Counts * ( 360.0 / (SwerveConstants::AngleGearRatio * 42.0))};
}

double SwerveModule::DegreesToNEO(units::degree_t Degrees){
    return Degrees.value() / (360.0 / (SwerveConstants::AngleGearRatio * 42.0));
}

units::meters_per_second_t SwerveModule::RPMToMPS(double VelocityCounts){
    double WheelRPM = VelocityCounts;
    units::meters_per_second_t WheelMPS = (WheelRPM * SwerveConstants::WheelCircumference) / 60_s;
    return WheelMPS;
}

void SwerveModule::SetDriveCANSparkMaxBusUsage(){
    if(m_EnableFollowing){
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10);
    } else {
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 500);
    }

    if(m_Usage == SwerveConstants::Usage::kAll){
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 20);
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 200);
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    } 
    if (m_Usage == SwerveConstants::Usage::kPositionOnly){
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500);
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 20);
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    } 
    if (m_Usage == SwerveConstants::Usage::kVeclocityOnly){
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500);
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 20);
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    } 
    if (m_Usage == SwerveConstants::Usage::kMinimal){
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500);
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 20);
        m_DriveMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    }
    
}

void SwerveModule::SetAngleCANSparkMaxBusUsage(){
    if(m_EnableFollowing){
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 10);
    } else {
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 500);
    }

    if(m_Usage == SwerveConstants::Usage::kAll){
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 20);
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 200);
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    } 
    if (m_Usage == SwerveConstants::Usage::kPositionOnly){
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500);
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 20);
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    } 
    if (m_Usage == SwerveConstants::Usage::kVeclocityOnly){
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500);
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 20);
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    } 
    if (m_Usage == SwerveConstants::Usage::kMinimal){
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500);
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 20);
        m_AngleMotor.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus3, 500);
    }
    
}

void SwerveModule::SetCANCoderBusUsage(){
    if(m_CCUsage == SwerveConstants::CCUsage::kAllCC){
        m_AngleCANcoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame_SensorData, 10);
        m_AngleCANcoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame_VbatAndFaults, 10);
    } 
    if (m_CCUsage == SwerveConstants::CCUsage::kSensorDataOnly){
        m_AngleCANcoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame_SensorData, 10);
        m_AngleCANcoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame_VbatAndFaults, 100);
    } 
    if (m_CCUsage == SwerveConstants::CCUsage::kFaultsOnly){
        m_AngleCANcoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame_SensorData, 100);
        m_AngleCANcoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame_VbatAndFaults, 10);
    } 
    if (m_CCUsage == SwerveConstants::CCUsage::kMinimalCC){
        m_AngleCANcoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame_SensorData, 100);
        m_AngleCANcoder.SetStatusFramePeriod(ctre::phoenix::sensors::CANCoderStatusFrame_VbatAndFaults, 100);
    }
}

void SwerveModule::SetCCUsage(int value){
    switch (value){
    
    case SwerveConstants::CCUsage::kSensorDataOnly:
        m_CCUsage = SwerveConstants::CCUsage::kSensorDataOnly;
        break;

    case SwerveConstants::CCUsage::kFaultsOnly:
        m_CCUsage = SwerveConstants::CCUsage::kFaultsOnly;
        break;

    case SwerveConstants::CCUsage::kMinimalCC:
        m_CCUsage = SwerveConstants::CCUsage::kMinimalCC;
        break;

    default:
        m_CCUsage = SwerveConstants::CCUsage::kAllCC;
        break;
    }

}

void SwerveModule::SetUsage(int value){
    switch (value){

    case SwerveConstants::Usage::kPositionOnly:
        m_Usage = SwerveConstants::Usage::kPositionOnly;
        break;
    
    case SwerveConstants::Usage::kVeclocityOnly:
        m_Usage = SwerveConstants::Usage::kVeclocityOnly;
        break;

    case SwerveConstants::Usage::kMinimal:
        m_Usage = SwerveConstants::Usage::kMinimal;
        break;

    default:
        m_Usage = SwerveConstants::Usage::kAll;
        break;
    }
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

// double SwerveModule::MPSToNEO(units::meters_per_second_t Velocity){
//     double WheelRPM = ( Velocity.value() * 60 ) / SwerveConstants::WheelCircumference.value();
//     double WheelVelocity = RPMToNEO(WheelRPM);
//     return WheelVelocity;
// }

