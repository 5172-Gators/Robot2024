// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;

public class Eject extends Command {

  Intake s_Intake;
  Kicker s_Kicker;

  /** Creates a new Eject. */
  public Eject(Intake intake, Kicker kicker) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.s_Intake = intake;
    this.s_Kicker = kicker;
    addRequirements(s_Intake, s_Kicker);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Intake.setIntakeArmPosition(Constants.Intake.deployedPosition);
    s_Intake.setIntakeSpeed(-0.5, -0.5);
    s_Kicker.setKickerRPM(-Constants.Kicker.kicker_intakeRPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.moveArm(0);
    s_Intake.stopIntake();
    s_Kicker.stopKicker();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
