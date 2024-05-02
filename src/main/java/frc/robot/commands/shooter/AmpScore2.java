// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.SetClimberPosition;
import frc.robot.commands.kicker.ShootAmp;
import frc.robot.commands.pitch.SetPitchPositionRaw;
import frc.robot.commands.turret.SetTurretAngle;
import frc.robot.subsystems.Climber.ClimbMode;


public class AmpScore2 extends SequentialCommandGroup {
  /** Creates a new AmpScore2. */
  public AmpScore2(BooleanSupplier fire, RobotContainer rc) {
    
    addCommands(
      new SetPitchPositionRaw(rc.s_Pitch, 1.77),
 
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new SetTurretAngle(rc.s_Turret, rc.s_Swerve, Constants.Turret.ampPosition),
          new InstantCommand(() -> rc.s_Climber.setClimbMode(ClimbMode.AMPSCORE)),
          new ParallelDeadlineGroup(
            new ShootAmp(fire, rc),
            new SetClimberPosition(Constants.Climber.ampScorePosition, rc.s_Climber, rc.s_Turret, true))
          ),

          new ParallelCommandGroup( new SetPitchPositionRaw(rc.s_Pitch, 1.77, true),
                                    new SetShooterRPMs(1550.0, 1550.0, rc.s_Shooter, true))),

      new SetTurretAngle(rc.s_Turret, rc.s_Swerve, Constants.Turret.intakePosition),
      new SetPitchPositionRaw(rc.s_Pitch, Constants.Pitch.intakePosition)
    );

  }
}
