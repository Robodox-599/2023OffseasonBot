// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.subsystem_LEDs;

public class command_FlashLEDs extends CommandBase {
  /** Creates a new command_FlashLEDs. */
  private Timer m_Timer = new Timer();
  private subsystem_LEDs m_LED;
  private double m_Duration;
  public command_FlashLEDs(subsystem_LEDs subsystem, double duration) {
    m_LED = subsystem;
    m_Duration = duration;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_LED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_LED.toggleFlashLEDs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_LED.toggleFlashLEDs();
    m_LED.setStandbyLED();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_Timer.get() > m_Duration;
  }
}
