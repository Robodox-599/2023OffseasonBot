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
  private CANdle m_CANdle;
  private Timer m_LEDTimer;
  private boolean m_FlashLEDs;
  private SingleFadeAnimation m_IntakedAnimation;
  private LEDState m_LEDState;
  private LEDState m_TriggerState;
  public subsystem_LEDs() {
    m_CANdle = new CANdle(LEDConstants.candleID);
    m_LEDTimer = new Timer();
    m_FlashLEDs = false;
    m_IntakedAnimation = new SingleFadeAnimation(0, 255, 0, 0, 0.5, 40);
    m_LEDState = LEDState.standby;
    m_TriggerState = LEDState.standby;

    m_LEDTimer.start();
    m_CANdle.configLEDType(LEDStripType.RGB);
    m_CANdle.configBrightnessScalar(0.5);
    m_CANdle.configLOSBehavior(true);
  }

  public LEDState getLEDState(){
    return m_LEDState;
  }

  public void setPurpleLED(){
    m_LEDState = LEDState.purple;
    m_TriggerState = LEDState.purple;
  }

  public void setYellowLED(){
    m_LEDState = LEDState.yellow;
    m_TriggerState = LEDState.yellow;
  }

  public void setStandbyLED(){
    m_LEDState = LEDState.standby;
  }

  public void toggleFlashLEDs(){
    m_FlashLEDs = !m_FlashLEDs;
  }

  public void setErrorLED(){
    m_LEDState = LEDState.error;
  }

  public void setOffLED(){
    m_LEDState = LEDState.off;
  }

  public void setTriggerStateLED(){
    m_LEDState = m_TriggerState;
  }

  public void cycleLED(){
    switch (m_LEDState){
      case purple:
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
    if (m_FlashLEDs && m_LEDTimer.get() % 10 > 5.0){
      m_CANdle.setLEDs(255, 255, 255);
    } else {
      switch(m_LEDState){
        case standby:
          m_CANdle.setLEDs(0, 255, 0);
          break;
        case purple:
          m_CANdle.setLEDs(255, 0, 255);
          break;
        case yellow:
          m_CANdle.setLEDs(255, 255, 0);
          break;
        case off:
          m_CANdle.setLEDs(0, 0, 255);
          break;
        case error:
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
