// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.intakeState;
import frc.robot.Constants.LEDConstants.LEDState;
import frc.robot.subsystems.subsystem_Intake;
import frc.robot.subsystems.subsystem_LEDs;

public class command_Intake extends CommandBase {
  /** Creates a new command_Intake. */
  subsystem_Intake m_Intake;
  subsystem_LEDs m_LEDs;
  Timer m_Timer = new Timer();
  public command_Intake(subsystem_Intake intake, subsystem_LEDs LEDs) {
    m_Intake = intake;
    m_LEDs = LEDs;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_LEDs.getLEDState() == LEDState.purple){
      m_Intake.setState(intakeState.cubeIntaking);
    } else if (m_LEDs.getLEDState() == LEDState.yellow){
      m_Intake.setState(intakeState.coneIntaking);
    }
    m_Timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_Intake.checkGamepieceIntake()){
      m_Timer.reset();
    } else {

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.setBrakeMode();
    if (m_Intake.getState() == intakeState.cubeIntaking){
      m_Intake.setState(intakeState.hasCube);
    } else {
      m_Intake.setState(intakeState.hasCone);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() > IntakeConstants.currentSpikeTimeSeconds;
  }
}
