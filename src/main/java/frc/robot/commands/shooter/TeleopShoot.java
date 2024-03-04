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

  public TeleopShoot(Shooter s_Shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Shooter = s_Shooter;

    // addRequirements(s_Shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean kickerSensorValue = s_Shooter.getKickerSensor();

    // sets the kicker speed if a note has been intaked
    // if (kickerSensorValue == false) {

    //   s_Shooter.setKicker(1, 1.0);
      
    // } else {
    //   s_Shooter.setKicker(0,0);
    // }

    // sets the shooter speed
    s_Shooter.setShooterRPM(0.4, 0.4);
    s_Shooter.setKickerRPM(0.55);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Shooter.setKickerRPM(0);
    s_Shooter.setShooterRPM(0,0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
    
  }
}
