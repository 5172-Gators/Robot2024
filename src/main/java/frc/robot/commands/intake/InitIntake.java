// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.climber.StowClimber;
import frc.robot.commands.pitch.SetPitchPosition;
import frc.robot.commands.turret.SetTurretPosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InitIntake extends ParallelCommandGroup {
  Intake s_Intake;
  Pitch s_Pitch;
  Turret s_Turret;
  
  /** Creates a new InitIntake. */
  public InitIntake(Intake intake, Pitch pitch, Turret turret) {
    s_Intake = intake;
    s_Pitch = pitch;
    s_Turret = turret;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new DeployIntake(s_Intake),
      new SetPitchPosition(s_Pitch, Constants.Pitch.intakePosition),
      new SetTurretPosition(s_Turret, Constants.Turret.R_intakingPosition)
    );
  }
}
