// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// For Rev BlinkIn
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */

  AddressableLED leds;
  AddressableLEDBuffer ledBuffer;

  private double flashPeriod = Constants.LEDs.kDefaultFlashPeriodSeconds;
  private double s_deadline = 0;
  private boolean lockModes = false;

  public enum LEDMode { SOLID, FLASH }
  public enum TimeMode { UNTIMED, TIMED }

  public LEDMode s_ledMode = LEDMode.SOLID;
  public TimeMode s_timeMode = TimeMode.UNTIMED;
  public Color s_currentColor = Color.kBlack;

  public LEDs() {

    leds = new AddressableLED(Constants.LEDs.ledPwmPort);
    
    ledBuffer = new AddressableLEDBuffer(Constants.LEDs.kNumLeds);
    leds.setLength(ledBuffer.getLength());

    leds.setData(ledBuffer);
    leds.start();
  }

  private void setColorInternal(int R, int G, int B) {
    for (int i = 0; i < ledBuffer.getLength(); i++)
      ledBuffer.setRGB(i, R, G, B);
    
    leds.setData(ledBuffer);
  }

  private void setColorInternal(Color color) {
    this.setColorInternal((int) (color.red*255.0), (int) (color.green*255.0), (int) (color.blue*255.0));
  }

  public void setColor(Color color) {
    if(!lockModes) {
      s_ledMode = LEDMode.SOLID;
      s_timeMode = TimeMode.UNTIMED;
      s_currentColor = color;
    }
  }

  public void setColor(Color color, LEDMode ledMode) {
    if(!lockModes) {
      s_timeMode = TimeMode.UNTIMED;
      s_currentColor = color;
      s_ledMode = ledMode;
    }
  }

  public void setColorTimed(Color color, LEDMode ledMode, double timeout) {
    if(!lockModes) {
      s_timeMode = TimeMode.TIMED;
      s_currentColor = color;
      s_ledMode = ledMode;
      s_deadline = Timer.getFPGATimestamp() + timeout;
    }
  }

  public void setFlashPeriod(double flashPeriod) {
    this.flashPeriod = flashPeriod;
  }

  public void resetFlashPeriodToDefault() {
    this.flashPeriod = Constants.LEDs.kDefaultFlashPeriodSeconds;
  }

  public void turnOff() {
    if(!lockModes) {
      s_timeMode = TimeMode.UNTIMED;
      s_currentColor = Color.kBlack;
      s_ledMode = LEDMode.SOLID;
      setColorInternal(Color.kBlack);
    }
  }

  @Override
  public void periodic() {
    // LED Controller based on requested time mode, requested color, and requested led mode

    if (s_timeMode == TimeMode.UNTIMED) {
      if (s_ledMode == LEDMode.SOLID)
        setColorInternal(s_currentColor);
      else if (s_ledMode == LEDMode.FLASH) {
        if ((Timer.getFPGATimestamp() % flashPeriod) < flashPeriod/2)
          setColorInternal(s_currentColor);
        else
          turnOff();
      }
    } else if (s_timeMode == TimeMode.TIMED && Timer.getFPGATimestamp() < s_deadline) {
      lockModes = true;
      if (s_ledMode == LEDMode.SOLID)
        setColorInternal(s_currentColor);
      else if (s_ledMode == LEDMode.FLASH) {
        if ((Timer.getFPGATimestamp() % flashPeriod) < flashPeriod/2)
          setColorInternal(s_currentColor);
        else
          turnOff();
      }
    } else if (Timer.getFPGATimestamp() >= s_deadline) {
      lockModes = false;
    } else {
      turnOff();
    }
  }
}
