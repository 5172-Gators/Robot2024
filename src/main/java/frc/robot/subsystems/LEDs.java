// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// For Rev BlinkIn
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Colors;



public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */

  AddressableLED leds;
  AddressableLEDBuffer ledBuffer;

  public LEDs() {

    leds = new AddressableLED(Constants.LEDs.ledPwmPort);
    
    ledBuffer = new AddressableLEDBuffer(Constants.LEDs.kNumLeds);
    leds.setLength(ledBuffer.getLength());

    leds.setData(ledBuffer);
    leds.start();
  }

  public void setColor(int R, int G, int B) {

    for (int i = 0; i < ledBuffer.getLength(); i++)
      ledBuffer.setRGB(i, R, G, B);
    
    leds.setData(ledBuffer);
  }

  public void setColor(Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++)
      ledBuffer.setRGB(i, (int) color.red*255, (int) color.green*255, (int) color.blue*255);
    
    leds.setData(ledBuffer);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
