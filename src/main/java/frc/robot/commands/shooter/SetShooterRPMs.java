// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class SetShooterRPMs extends Command {
  /** Creates a new SetShooterRPMs. */
  Shooter s_Shooter;

  double leftRPM;
  double rightRPM;

  boolean dontFinish = false;

  public SetShooterRPMs(double leftRPM, double rightRPM, Shooter shooter, boolean dontFinish) {
    this.s_Shooter = shooter;
    this.leftRPM = leftRPM;
    this.rightRPM = rightRPM;
    this.dontFinish = dontFinish;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(s_Shooter);
  }

  public SetShooterRPMs(double leftRPM, double rightRPM, Shooter shooter) {
    this(leftRPM, rightRPM, shooter, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    s_Shooter.setShooterRPM(rightRPM, leftRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Shooter.isReady() && !dontFinish;
  }
}
