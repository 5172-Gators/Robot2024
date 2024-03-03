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

  Spark s_spark;
  // double strobeRed = (-0.11); 
  // double rainbow = (-0.99);   //fancy shit
  // double red = (0.61);    //outtaking
  // double darkGreen = (0.75);
  // double lime = (0.73);   
  // double green = (0.77);  //intaking
  // double aqua = (0.81);   
  // double blue = (0.87);   // ready to shoot
  // double strobeBlue = (-0.09); // looking for target


  public LEDs(){
    s_spark = new Spark(0);
   
   
    //spark.set(Constants.LEDs.green);
  
  
  
  }

  public void setColor (double color){

    s_spark.set(color);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
