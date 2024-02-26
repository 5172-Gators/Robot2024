// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pitch;

public class SetPitchPosition extends Command {
  /** Creates a new SetPitchPosition. */

  Pitch s_Pitch;
  double position;

  double currentPosition;

  public SetPitchPosition(Pitch s_Pitch, double position) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Pitch = s_Pitch;
    this.position = position;

    addRequirements(s_Pitch);

    this.currentPosition = s_Pitch.getPosition();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (currentPosition < position){

      s_Pitch.movePitch(Math.abs(position - currentPosition) / -0.1);
      

    } else if (currentPosition > position){

      s_Pitch.movePitch(Math.abs(position - currentPosition) / 0.1);
      

    } else if (currentPosition == position + Constants.Pitch.allowableError || currentPosition == position - Constants.Pitch.allowableError){

      s_Pitch.movePitch(0);

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    if(currentPosition == position){

      return true;

    } else {

      return false;

    }

  }

}


