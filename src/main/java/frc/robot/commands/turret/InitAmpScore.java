// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.turret;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.climber.JankyClimberPosition;
import frc.robot.commands.climber.SetClimbMode;
import frc.robot.commands.pitch.SetPitchPositionRaw;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimbMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class InitAmpScore extends SequentialCommandGroup {
  /** Creates a new InitAmpScore. */

  Pitch s_Pitch;
  Turret s_Turret;
  Climber s_Climber;

  public InitAmpScore(Pitch pitch, Turret turret) {

    this.s_Pitch = pitch;
    this.s_Turret = turret;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(new SetPitchPositionRaw(s_Pitch, Constants.Pitch.ampScoreTravelPosition),
                new SetTurretPosition(s_Turret, Constants.Turret.turretAmpPosition));
  }
}
