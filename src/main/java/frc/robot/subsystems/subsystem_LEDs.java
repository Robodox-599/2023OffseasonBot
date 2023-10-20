// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.LEDState;

import com.ctre.phoenix.led.CANdle;
//import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
//import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;

public class subsystem_LEDs extends SubsystemBase {
  /** Creates a new subsystem_LEDs. */
  private boolean flashLEDs = false;
  SingleFadeAnimation IntakedAnimation = new SingleFadeAnimation(0, 255, 0, 0, 0.5, 40);
  CANdle m_CANdle;
  Timer m_LEDTimer;
  LEDState m_LEDState = LEDState.Standby;
  LEDState m_TriggerState = LEDState.Standby;
  public subsystem_LEDs() {
    m_CANdle = new CANdle(LEDConstants.CANdleID);
    m_LEDTimer = new Timer();

    m_LEDTimer.start();
    m_CANdle.configLEDType(LEDStripType.RGB);
    m_CANdle.configBrightnessScalar(0.5);
    m_CANdle.configLOSBehavior(true);
  }

  public LEDState getLEDState(){
    return m_LEDState;
  }

  public void setPurpleLED(){
    m_LEDState = LEDState.Purple;
    m_TriggerState = LEDState.Purple;
  }

  public void setYellowLED(){
    m_LEDState = LEDState.Yellow;
    m_TriggerState = LEDState.Yellow;
  }

  public void setStandbyLED(){
    m_LEDState = LEDState.Standby;
  }

  public void toggleFlashLEDs(){
    flashLEDs = !flashLEDs;
  }

  public void setErrorLED(){
    m_LEDState = LEDState.Error;
  }

  public void setOffLED(){
    m_LEDState = LEDState.Off;
  }

  public void setTriggerStateLED(){
    m_LEDState = m_TriggerState;
  }

  public void cycleLED(){
    switch (m_LEDState){
      case Purple:
        setYellowLED();
        break;
      default:
        setPurpleLED();
        break;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (flashLEDs && m_LEDTimer.get() % 10 > 5.0){
      m_CANdle.setLEDs(255, 255, 255);
    } else {
      switch(m_LEDState){
        case Standby:
          m_CANdle.setLEDs(0, 255, 0);
          break;
        case Purple:
          m_CANdle.setLEDs(255, 0, 255);
          break;
        case Yellow:
          m_CANdle.setLEDs(255, 255, 0);
          break;
        case Off:
          m_CANdle.setLEDs(0, 0, 255);
          break;
        case Error:
        default:
          m_CANdle.setLEDs(0, 255, 0);
          break;
      }
    }
  }

  public CommandBase cycleLEDCommand(){
      // implicitly require `this`
      return this.runOnce(() -> cycleLED());
  }

}
