// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_DriveTank.h"

subsystem_DriveTank::subsystem_DriveTank(): m_LeftBackMotor{MotorConstants::LBMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                            m_RightBackMotor{MotorConstants::RBMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless}, 
                                            m_LeftFrontMotor{MotorConstants::LFMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                            m_RightFrontMotor{MotorConstants::RFMotorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                         m_LeftFrontMotorPID{m_LeftFrontMotor.GetPIDController()}, 
                         m_RightFrontMotorPID{m_RightFrontMotor.GetPIDController()}, 
                         m_LeftFrontEncoder{m_LeftFrontMotor.GetEncoder()},
                         m_RightFrontEncoder{m_RightFrontMotor.GetEncoder()}

{
    m_LeftFrontMotorPID.SetP(MotorConstants::MotorP);
    m_LeftFrontMotorPID.SetI(MotorConstants::MotorI);
    m_LeftFrontMotorPID.SetD(MotorConstants::MotorD);
    m_RightFrontMotorPID.SetP(MotorConstants::MotorP);
    m_RightFrontMotorPID.SetI(MotorConstants::MotorI);
    m_RightFrontMotorPID.SetD(MotorConstants::MotorD);
    m_LeftFrontEncoder.SetPosition(0);
    m_RightFrontEncoder.SetPosition(0);

    // m_Motor.SetVoltage(units::voltage::volt_t (12));
    // m_Motor.SetSmartCurrentLimit(5, 7, 2650/9);

    
}

frc2::CommandPtr subsystem_DriveTank::Drive(double DriveSpeed){
DriveSpeed = DesiredSpeed;
}
frc2::CommandPtr subsystem_DriveTank::Turn(double TurnPos){
TurnPos=DesiredTurnPosition;
}
bool subsystem_DriveTank::IsLeftAngleAtDesired(){
    if(fabs(DesiredTurnPosition - LeftEncValue) < MotorConstants::BufferZone){
        return true;
    }else{
        return false;}

}
bool subsystem_DriveTank::IsRightAngleAtDesired(){
    if(fabs(DesiredTurnPosition - RightEncValue) < MotorConstants::BufferZone){
        return true;
    }else{
        return false;}

}

// This method will be called once per scheduler run
void subsystem_DriveTank::Periodic() {
    LeftEncValue=m_LeftFrontEncoder.GetPosition();
    RightEncValue=m_RightFrontEncoder.GetPosition();

    if (!IsLeftAngleAtDesired()){
       // frc::SmartDashboard::SmartDashboard::PutNumber("desiredwristposition", DesiredWristPosition);
        m_LeftFrontMotorPID.SetReference(DesiredTurnPosition, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);
       // frc::SmartDashboard::SmartDashboard::PutBoolean("went into if statement", true);
    }
        if (!IsRightAngleAtDesired()){
       // frc::SmartDashboard::SmartDashboard::PutNumber("desiredwristposition", DesiredWristPosition);
        m_RightFrontMotorPID.SetReference(DesiredTurnPosition, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);
       // frc::SmartDashboard::SmartDashboard::PutBoolean("went into if statement", true);
    }
//change 0.01 if u want to have less differential-ness !!!!!
if (DesiredTurnPosition>0){
m_LeftBackMotor.Set(DesiredSpeed*MotorConstants::PercentMaxOutput+(0.01*DesiredTurnPosition/MotorConstants::TurnGearRatio ));
m_RightBackMotor.Set(DesiredSpeed*MotorConstants::PercentMaxOutput-(0.01*DesiredTurnPosition/MotorConstants::TurnGearRatio));
} else if (DesiredTurnPosition<0){
m_LeftBackMotor.Set(DesiredSpeed*MotorConstants::PercentMaxOutput-(0.01*DesiredTurnPosition/MotorConstants::TurnGearRatio));
m_RightBackMotor.Set(DesiredSpeed*MotorConstants::PercentMaxOutput+(0.01*DesiredTurnPosition/MotorConstants::TurnGearRatio));
}else {
    m_LeftBackMotor.Set(DesiredSpeed*MotorConstants::PercentMaxOutput);
    m_RightBackMotor.Set(DesiredSpeed*MotorConstants::PercentMaxOutput);
}
//  m_LeftBackMotor.Set(DesiredSpeed*MotorConstants::PercentMaxOutput);
//     m_RightBackMotor.Set(DesiredSpeed*MotorConstants::PercentMaxOutput);
//!!!!!!! if diffy dont work^

}
