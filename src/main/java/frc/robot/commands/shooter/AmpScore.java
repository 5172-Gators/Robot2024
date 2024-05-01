// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Shooter;
import frc.robot.AimingTolerances;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.climber.SetClimbMode;
import frc.robot.commands.climber.SetClimberPosition;
import frc.robot.commands.kicker.ShootAmp;
import frc.robot.commands.pitch.SetPitchPositionRaw;
import frc.robot.commands.turret.InitAmpScore;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Climber.ClimbMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpScore extends SequentialCommandGroup {
  /** Creates a new ImprovedAmpScore. */

  public AmpScore(BooleanSupplier fire, RobotContainer rc) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InitAmpScore(rc.s_Pitch, rc.s_Turret),
                new ParallelCommandGroup(new SetClimbMode(ClimbMode.AMPSCORE, rc.s_Climber),
                                         new SetPitchPositionRaw(rc.s_Pitch, 1.77),
                                         new SetShooterRPMs(1550.0, 1550.0, rc.s_Shooter)),
                                         new ShootAmp(fire, rc));
                
  }
}
