// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class LEDDefaultCommand extends Command {

  LEDs s_LED;
  Shooter s_Shooter;
 

  /** Creates a new LEDTimedCommand. */
  public LEDDefaultCommand(LEDs LED, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
 
    this.s_LED = LED;
    this.s_Shooter = shooter;

    addRequirements(s_LED);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    // s_LED.setColor(0.67);
    if (!s_Shooter.getKickerSensor()){

      s_LED.setColor(Color.kOrange);

    } else {

      s_LED.setColor(Color.kBlack);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_LED.setColor(Color.kBlack);
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
