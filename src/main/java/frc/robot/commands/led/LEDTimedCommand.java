// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shooter;

public class LEDTimedCommand extends Command {

  LEDs s_LED;
  double startTime;
  double duration;
  double signal;
  double currentTime;

  /** Creates a new LEDTimedCommand. */
  public LEDTimedCommand(double signal, double duration, LEDs LED) {
    // Use addRequirements() here to declare subsystem dependencies.
 
    this.s_LED = LED;

    this.signal = signal;
    this.duration = duration;

    addRequirements(s_LED);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    startTime = Timer.getFPGATimestamp();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_LED.setColor(this.signal);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_LED.setColor(0.99);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    currentTime = Timer.getFPGATimestamp();
    return (currentTime - startTime > this.duration);
  }
}
