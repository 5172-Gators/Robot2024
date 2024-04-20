// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.NotePossession;

public class KickerDefaultCommand extends Command {

  Kicker s_Kicker;
  Shooter s_Shooter;

  /** Creates a new KickerDefaultCommand. */
  public KickerDefaultCommand(Kicker kicker, Shooter shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Kicker = kicker;
    this.s_Shooter = shooter;
    addRequirements(s_Kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Shooter.currentNotePossession == NotePossession.HALF && !s_Shooter.shooterSensorFlag) {
      s_Kicker.setKickerRPM(Constants.Kicker.kicker_creepRPM);
    }
    if (s_Shooter.currentNotePossession == NotePossession.FULL){
      s_Kicker.setKickerRPM(-Constants.Kicker.kicker_creepRPM);
    }
    if (s_Shooter.shooterSensorFlag && s_Shooter.currentNotePossession == NotePossession.HALF){
      s_Kicker.stopKicker();
    }
    if (s_Shooter.currentNotePossession == NotePossession.NONE){
      s_Kicker.stopKicker();
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
