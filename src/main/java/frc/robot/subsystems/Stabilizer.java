// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Stabilizer extends SubsystemBase {
  /** Creates a new Stabilizer. */
  
  Servo climberServo;
 
  public Stabilizer() {

    climberServo = new Servo(2);

  }

  public void stabilizerControl(double position){

    climberServo.set(position);
  }

  public void setBounds(int min, int max) {
    climberServo.setBoundsMicroseconds(max, 0, (max+min)/2, 0, min);
  }

  public void setBounds(int min, int mid, int max) {
    climberServo.setBoundsMicroseconds(max, 0, mid, 0, min);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
