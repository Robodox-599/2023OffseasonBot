// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Wrist.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include <units/voltage.h>

subsystem_Wrist::subsystem_Wrist(): m_WristMotor{WristConstants::motorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless}, 
                         m_WristPID{m_WristMotor.GetPIDController()}, 
                         m_WristEncoder{m_WristMotor.GetEncoder()}

{
    m_WristPID.SetP(WristConstants::kWristP);
    m_WristPID.SetI(WristConstants::kWristI);
    m_WristPID.SetD(WristConstants::kWristD);
    m_WristEncoder.SetPosition(WristConstants::start_pos);
    m_WristMotor.SetVoltage(units::voltage::volt_t (12));
    m_WristMotor.SetSmartCurrentLimit(5, 7, 2650/9);

    
}
// This method will be called once per scheduler run

frc2::CommandPtr subsystem_Wrist::ResetWrist(){

    return RunOnce([this]{return m_WristEncoder.SetPosition(WristConstants::start_pos);});
}
frc2::CommandPtr subsystem_Wrist::MoveWristCommand(double desiredPos){
    return RunOnce([this, desiredPos]{return SetWristByPosition(desiredPos);});
    
}
frc2::CommandPtr subsystem_Wrist::ZeroWrist(){
    return RunOnce([this]{return SetWristByPosition(0);});
}

//call Reset Wrist and zero wrist hand in hand !!!!!

frc2::CommandPtr subsystem_Wrist::ManualWrist(double StickPos){
    return RunOnce([this,StickPos]{return SetWristByPosition(WristEnc+(StickPos*WristConstants::stepperConstant));});
} 

//!!!!!!!!!!!!!!!!!!!1 come back and fix
// frc2::CommandPtr subsystem_Wrist::HighCone(){
//         MoveWristCommand(WristPositions::HighConePos);
// }
// frc2::CommandPtr subsystem_Wrist::HighCube(){
// MoveWristCommand(WristPositions::HighCubePos);
// }
// frc2::CommandPtr subsystem_Wrist::MidCone(){
// MoveWristCommand(WristPositions::MidConePos);
// }
// frc2::CommandPtr subsystem_Wrist::MidCube(){
// MoveWristCommand(WristPositions::MidCubePos);
// }
// frc2::CommandPtr subsystem_Wrist::Substation(){
// MoveWristCommand(WristPositions::SubstationPos);
// }

bool subsystem_Wrist::IsWristAtDesiredPosition(){
    if(fabs(DesiredWristPosition - WristEnc) < WristConstants::bufferZone){
        return true;
    }else{
        return false;}
}
bool subsystem_Wrist::WristThreshold(double Threshold){
    if(DesiredWristPosition >= WristEnc){
        return WristEnc > Threshold;
    } else{
        return WristEnc < Threshold;
    }
}
void subsystem_Wrist::SetWristByPosition(double tiltPos){
    DesiredWristPosition = tiltPos;
    frc::SmartDashboard::SmartDashboard::PutNumber("tilt pos", tiltPos);
}

void subsystem_Wrist::Periodic() {

    // double relWristAngle = 180 - ( (m_WristAbsEncoder.GetAbsolutePosition() - 0.475) * 83.7 )/ 0.2325;
    // // frc::SmartDashboard::PutNumber("RelWristAngle", relWristAngle);
    // double AbsWristAngle = 180 - (relWristAngle - ElbowAngle);
    // // frc::SmartDashboard::PutNumber("absWristAngle", AbsWristAngle);

    // double AbsWristPos = AbsWristAngle  * 83.7 / 360;
    //confused

    WristEnc = m_WristEncoder.GetPosition();
    frc::SmartDashboard::SmartDashboard::PutNumber("Wrist Pos", WristEnc);
    m_Angle = (m_WristEncoder.GetPosition() / 0.2325  );
    if (!IsWristAtDesiredPosition()){
        frc::SmartDashboard::SmartDashboard::PutNumber("desiredwristposition", DesiredWristPosition);
        m_WristPID.SetReference(DesiredWristPosition, rev::CANSparkMaxLowLevel::ControlType::kPosition, 0);
        frc::SmartDashboard::SmartDashboard::PutBoolean("went into if statement", true);
    } else {         
        frc::SmartDashboard::SmartDashboard::PutBoolean("went into if statement", false);
    }

    frc::SmartDashboard::SmartDashboard::PutNumber("current encoder val", WristEnc);

}
