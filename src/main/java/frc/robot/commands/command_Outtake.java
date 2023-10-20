// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants.intakeState;
import frc.robot.subsystems.subsystem_Intake;
import frc.robot.subsystems.subsystem_LEDs;

public class command_Outtake extends CommandBase {
  /** Creates a new command_Outtake. */
  subsystem_Intake m_Intake;
  subsystem_LEDs m_LEDs;
  Timer m_Timer = new Timer();
  public command_Outtake(subsystem_Intake intake, subsystem_LEDs LEDs) {
    m_Intake = intake;
    m_LEDs = LEDs;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_Intake);
    addRequirements(m_LEDs);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Intake.setCoastMode();
    if (m_Intake.getState() == intakeState.hasCube){
      m_Intake.setState(intakeState.cubeOuttaking);
    } else {
      m_Intake.setState(intakeState.coneOuttaking);
    }
    m_Timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Intake.setState(intakeState.noGamepiece);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() > 0.5;
  }
}
