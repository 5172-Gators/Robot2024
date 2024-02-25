// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Shooter;

public class TeleopShoot extends Command {
  /** Creates a new shoot. */

  Shooter s_Shooter;

  double rightSpeed;
  double leftSpeed;
  double kickerSpeed;

  boolean sensorValue;
  
  // speed of the motor will always be 1

  public TeleopShoot(Shooter s_Shooter, double rightSpeed, double leftSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Shooter = s_Shooter;

    addRequirements(s_Shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // sets the shooter speed
    s_Shooter.setShooter(1, 0.8, 0.4);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
