// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/subsystem_Elevator.h"
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>

subsystem_Elevator::subsystem_Elevator() : m_ElevatorMotor{ElevatorConstants::kElevatorID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                            m_ElevatorFollower{ElevatorConstants::kElevatorFollowerID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
                                            m_ElevatorEncoder{ElevatorConstants::kEncoderID},
                                            m_ElevatorMotorPID{m_ElevatorMotor.GetPIDController()},
                                            m_ElevatorFollowerPID{m_ElevatorFollower.GetPIDController()},
                                            m_MotorEncoder{m_ElevatorMotor.GetEncoder()},
                                            m_MotorFollowerEncoder{m_ElevatorFollower.GetEncoder()}

                                            
{
    m_ElevatorMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    m_ElevatorFollower.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    
    m_ElevatorMotor.SetSmartCurrentLimit(5,7,117.157);
    m_ElevatorFollower.SetSmartCurrentLimit(5,7,117.157);
    //5 sec
    
    //latency or whatever
    m_ElevatorFollower.Follow(m_ElevatorMotor);

    m_ElevatorMotorPID.SetP(0.1);
    m_ElevatorMotorPID.SetI(0);
    m_ElevatorMotorPID.SetD(0.0005);

    m_ElevatorFollowerPID.SetP(ElevatorConstants::kElevatorP);
    m_ElevatorFollowerPID.SetI(ElevatorConstants::kElevatorI);
    m_ElevatorFollowerPID.SetD(ElevatorConstants::kElevatorD);

    SetElevatorPosition(ElevatorConstants::kElevatorStow);
    frc::SmartDashboard::SmartDashboard::PutNumber("motor current", m_ElevatorMotor.GetOutputCurrent());
    frc::SmartDashboard::SmartDashboard::PutNumber("follower motor current", m_ElevatorFollower.GetOutputCurrent());
    //m_ElevatorEncoder.SetDistancePerRotation(5.5);
}

// // Will merge the command ptrs in Cgroup later w saliks wrist
// frc2::CommandPtr subsystem_Elevator::ToMidCone()
// {
//     MoveElevator(ElevatorConstants::kElevatorScoreMidCone);
//     //its givin error cuzzzz i didnt return anything to ptr
//     //return frc2::cmd::Sequence(MoveElevator(ElevatorConstants::kElevatorScoreMidCone));
//     // return frc2::cmd::Sequence()
// }

// frc2::CommandPtr subsystem_Elevator::ToHighCone()
// {
//     MoveElevator(ElevatorConstants::kElevatorScoreHighCone);
// }   

// frc2::CommandPtr subsystem_Elevator::ToMidCube()
// {
//     MoveElevator(ElevatorConstants::kElevatorScoreMidCube);
// }

// frc2::CommandPtr subsystem_Elevator::ToHighCube()
// {
//     MoveElevator(ElevatorConstants::kElevatorScoreHighCube);
// }

// frc2::CommandPtr subsystem_Elevator::ToStow()
// {
//     MoveElevator(ElevatorConstants::kElevatorStow);
// }

// frc2::CommandPtr subsystem_Elevator::ToHybrid()
// {
//     MoveElevator(ElevatorConstants::kElevatorScoreHybrid);
// }

// frc2::CommandPtr subsystem_Elevator::ToDoubleSubstation()
// {
//     MoveElevator(ElevatorConstants::kElevatorScoreDoubleSubstation);
// }

//  : m_ElevatorEncoder(frc::DutyCycleEncoder::DutyCycleEncoder(int channel))
// void subsystem_Elevator::ElevatorEncoder()
// {
//     //initializing elevator encoder

// }

// void subsystem_Elevator::MoveElevator(double angle)
// {
    
//     //does follower need to be updated here if it .follows
//     //move set reference to periodic later if this doesn't work
//     // also change to rotations if it dont workout
//     //not using motionmagic cuz inertia of weight of gamepiece messes up so use sid
//     m_ElevatorMotorPID.SetReference(angle, rev::CANSparkMax::ControlType::kPosition, 0);
//     m_ElevatorFollowerPID.SetReference(angle, rev::CANSparkMax::ControlType::kPosition, 0);
    
// }

void subsystem_Elevator::ResetElevatorEncoder()
{
    m_ElevatorEncoder.Reset();
    //frc::SmartDashboard::SmartDashboard::PutBoolean("Reset Elevator Encoder", true);

}

void subsystem_Elevator::SetElevatorPosition(double ElevatorPositionPassedIn)
{
    DesiredElevatorPosition = ElevatorPositionPassedIn;
    m_ElevatorMotorPID.SetReference(DesiredElevatorPosition, rev::CANSparkMax::ControlType::kPosition);
    //m_ElevatorFollowerPID.SetReference(DesiredElevatorPosition, rev::CANSparkMax::ControlType::kPosition, 0);
    frc::SmartDashboard::SmartDashboard::PutNumber("Elevator Position Passed in", ElevatorPositionPassedIn);
    //m_ElevatorFollowerPID.SetReference(DesiredElevatorPosition, rev::CANSparkMax::ControlType::kPosition, 0);
}

void subsystem_Elevator::MoveElevatorManually(double axis)
{
    // manualX = armX + (leftAxis * ArmConstants::JoystickToArm);
    // manualY = armY + (rightAxis * ArmConstants::JoystickToArm);
    // MoveArm(manualX, manualY);

    DesiredElevatorPosition += axis * ElevatorConstants::kManualSpeed;

    

    m_ElevatorMotorPID.SetReference(DesiredElevatorPosition, rev::CANSparkMax::ControlType::kPosition);
    //m_ElevatorFollowerPID.SetReference(DesiredElevatorPosition, rev::CANSparkMax::ControlType::kPosition, 0);
}

double subsystem_Elevator::GetElevatorPosition() 
{
    return m_ElevatorEncoder.GetDistance();
}

double subsystem_Elevator::GetElevatorsMotorPosition()
{
    return m_MotorEncoder.GetPosition();
}

void subsystem_Elevator::SetElevatorMotorSpeed(double speed)
{
    m_ElevatorMotor.Set(speed);
    m_ElevatorFollower.Set(speed);
}

// this part is diff from wrist???
bool subsystem_Elevator::IsElevatorAtDesiredPosition()
{
    // if(fabs(DesiredElevatorPosition - ElevatorEnc) < ElevatorConstants::kBufferZone)
    // {
    //     return true;
    // }
    // return false;

    if(fabs(DesiredElevatorPosition - ElevatorEnc) < ElevatorConstants::kBufferZone)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool subsystem_Elevator::ElevatorThreshold(double Threshold)
{
    if(DesiredElevatorPosition >= ElevatorEnc)
    {
        return ElevatorEnc > Threshold;
    }
    else
    {
        return ElevatorEnc < Threshold;
    }

    //return DesiredElevatorPosition >= 0.0 ? ElevatorEnc > Threshold : ElevatorEnc < Threshold;
}

// This method will be called once per scheduler run
void subsystem_Elevator::Periodic() {
    //m_motor2.set(m_motor1.get());
    ElevatorEnc = m_MotorEncoder.GetPosition();
    if (!IsElevatorAtDesiredPosition())
    {
        frc::SmartDashboard::SmartDashboard::PutNumber("Desired Elevator Position", DesiredElevatorPosition);
        m_ElevatorMotorPID.SetReference(DesiredElevatorPosition, rev::CANSparkMax::ControlType::kPosition);
        //m_ElevatorFollowerPID.SetReference(DesiredElevatorPosition, rev::CANSparkMax::ControlType::kPosition, 0);
        frc::SmartDashboard::SmartDashboard::PutBoolean("Went into if statement", true);
    }
    else
    {
        frc::SmartDashboard::SmartDashboard::PutBoolean("Went into if statement", false);
    }

    frc::SmartDashboard::SmartDashboard::PutNumber("Current motor Encoder Value", ElevatorEnc);


}
