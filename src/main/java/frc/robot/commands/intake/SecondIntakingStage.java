// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SecondIntakingStage extends Command {
  /** Creates a new SecondIntakingStage. */

  Shooter s_Shooter;

  boolean shooterSensorValue;
  
  public SecondIntakingStage(Shooter s_Shooter) {
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

    shooterSensorValue = s_Shooter.getShooterSensor();

    if (shooterSensorValue == true){

      s_Shooter.setKicker(1, 0.1);

    } else {

      s_Shooter.setKicker(0, 0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return false;
  }
}
