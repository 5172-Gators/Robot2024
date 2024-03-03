// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkFlex;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Kicker extends SubsystemBase {
  /** Creates a new Kicker. */

  CANSparkFlex kicker;
  
  public Kicker() {

    kicker = new CANSparkFlex(Constants.Kicker.kickerMotorID, MotorType.kBrushless);
    kicker.restoreFactoryDefaults();
    kicker.setInverted(true);

  }

  public void runKicker(double speed){

    kicker.set(speed);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
