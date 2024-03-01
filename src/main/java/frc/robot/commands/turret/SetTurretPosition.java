// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class SetTurretPosition extends Command {
  /** Creates a new SetTurretPosition. */

  Turret s_Turret;
  double position;

  public SetTurretPosition(Turret s_Turret, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    
    this.s_Turret = s_Turret;
    this.position = position;
    addRequirements(s_Turret);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Turret.setPosition(position);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (s_Turret.getRotatePosition() > position - Constants.Turret.allowableError || s_Turret.getRotatePosition() < position + Constants.Turret.allowableError){

      return true;

    } else {

      return false;

    }
  }
}