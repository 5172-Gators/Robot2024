// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.kicker;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Shooter;

public class ZeroNote extends Command {

  Kicker s_Kicker;
  Shooter s_Shooter;

  int state = 0;

  /** Creates a new ZeroNote. */
  public ZeroNote(Kicker kicker, Shooter shooter) {
    this.s_Kicker = kicker;
    this.s_Shooter = shooter;
    addRequirements(s_Kicker, s_Shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    state = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if (state == 0) {
    //   // if (s_Shooter.shooterIsReady())
    //   s_Shooter.setShooterRPM(-Constants.Shooter.creepRPM, -Constants.Shooter.creepRPM);
    //   s_Kicker.setKickerRPM(Constants.Kicker.kicker_intakeRPM);

    //   if (s_Shooter.getShooterSensor() == false) // shooter sensor
    //     state = 1;
    // }

    if (state == 0) {
      s_Kicker.setKickerRPM(-Constants.Kicker.kicker_creepRPM);
      s_Shooter.setShooterRPM(-Constants.Shooter.creepRPM, -Constants.Shooter.creepRPM);

      if (s_Shooter.getShooterSensor() == true)
        state = 1;
    }

    if (state == 1) {
      s_Kicker.setKickerRPM(Constants.Kicker.kicker_creepRPM);

      if (s_Shooter.getShooterSensor() == false)
        state = 2;
    }

    if (state == 2) {
      s_Kicker.setKickerRPM(Constants.Kicker.kicker_creepRPM);

      if (s_Shooter.getShooterSensor() == false)
        state = 3;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Kicker.stopKicker();
    s_Shooter.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (state == 3);
  }
}
