// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trap_scorer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TrapScorer;
import frc.robot.subsystems.Turret;

public class LoadTrapScorer extends Command {
  /** Creates a new LoadTrapScorer. */

  int state;

  Shooter s_Shooter;
  TrapScorer s_TrapScorer;
  Turret s_Turret;
  Pitch s_Pitch;
  
  public LoadTrapScorer(Shooter shooter, TrapScorer trapScorer, Turret turret, Pitch pitch) {

    s_Shooter = shooter;
    s_TrapScorer = trapScorer;
    s_Turret = turret;
    s_Pitch = pitch;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    state = 0;

    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
    s_Shooter.stopShooter();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // state 0
    if (!s_Shooter.getKickerSensor() && !s_Shooter.getShooterSensor()){

      s_Pitch.setPosition(Constants.Pitch.loadingPosition);
      s_TrapScorer.setElevatorPosition(Constants.TrapScorer.loadingPosition);


      if (s_Pitch.isReady() && s_TrapScorer.elevatorisAtPosition())
        state = 1;

    }

    // state 1
    if (state == 1){

      s_TrapScorer.setScoringWheelsRPM(Constants.TrapScorer.loadingRPM);
      s_Shooter.setShooterRPM(700, 700);

      if (s_Shooter.getKickerSensor() && s_Shooter.getShooterSensor()){

        s_TrapScorer.stopScoringWheels();
        s_Shooter.stopShooter();

        state = 2;
      }
    } 

    // state 2
    if (state == 2){

      s_TrapScorer.setElevatorPosition(Constants.TrapScorer.stowedPosition);

      if (s_TrapScorer.elevatorisAtPosition()){

        state = 3;
      }

    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_TrapScorer.setElevatorPosition(Constants.TrapScorer.stowedPosition);
    s_TrapScorer.stopScoringWheels();

    state = 0;
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
   
    return state == 3;

  }
}
