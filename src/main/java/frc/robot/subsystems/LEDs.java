// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// For Rev BlinkIn
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Colors;



public class LEDs extends SubsystemBase {
  /** Creates a new LEDs. */

  Spark ledController;

  public LEDs() {

    ledController = new Spark(0);
  
  }

  public void setColor(double color) {

    ledController.set(color);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
