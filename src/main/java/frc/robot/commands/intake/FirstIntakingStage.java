// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.*;


public class FirstIntakingStage extends Command {
  /** Creates a new FirstIntakingStage. */
  Shooter s_Shooter;

  boolean kickerSensorValue;
  boolean shooterSensorValue;

  public FirstIntakingStage(Shooter s_Shooter) {
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

    kickerSensorValue = s_Shooter.getKickerSensor();
    shooterSensorValue = s_Shooter.getShooterSensor();

    if (kickerSensorValue == true){

      s_Shooter.setKicker(1, 0.55);

    } else if (shooterSensorValue == false){

      s_Shooter.setKicker(-1, 0.1);

    } else if (shooterSensorValue == true && kickerSensorValue == false) {

      s_Shooter.setKicker(0, 0);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if (shooterSensorValue == true && kickerSensorValue == false) {

      return true;

    } else {

      return false;

    }
    
  }
}
