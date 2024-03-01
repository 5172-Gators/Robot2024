// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LimelightPosition;
import frc.robot.subsystems.*;

public class AlignToAprilTag extends Command {
  /** Creates a new alignAprilTag. */

  Limelight s_Limelight;
  Turret s_Turret;
  Pitch s_Pitch;
  Swerve s_Swerve;
  LimelightPosition desiredPosition;

  public AlignToAprilTag(Limelight m_Limelight, Turret m_Turret, Pitch s_Pitch, LimelightPosition desiredPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Limelight =  m_Limelight;
    this.s_Turret =  m_Turret;
    this.s_Pitch = s_Pitch;
    this.s_Swerve = s_Swerve;
    this.desiredPosition = desiredPosition;
    
    addRequirements(m_Limelight, m_Turret, s_Pitch);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double currentX = s_Limelight.getTX();
    double currentY = s_Limelight.getTY();
    double currentTA = s_Limelight.getTA();

    /* (hopefully) moves the turret to a desired tX value */
    if (desiredPosition.getTX() > currentX){

      s_Turret.rotateTurret(0.5);
 
    } else if (desiredPosition.getTX() < currentX) {

      s_Turret.rotateTurret(-0.5);

    } else if (desiredPosition.getTX() == currentX + Constants.Limelight.allowableError || desiredPosition.getTX() == currentX - Constants.Limelight.allowableError){

      s_Turret.rotateTurret(0);

    }

    /* (hopefully) moves the pitch up and down to meet a desired tY value */
    if (desiredPosition.getTY() > currentY){

      s_Pitch.movePitch(-0.3);

    } else if (desiredPosition.getTY() < currentY){

      s_Pitch.movePitch(0.3);

    } else if (desiredPosition.getTY() == currentY - Constants.Limelight.allowableError || desiredPosition.getTY() == currentY + Constants.Limelight.allowableError){

      s_Pitch.movePitch(0);

    }

  if (desiredPosition.getTA() < currentTA){

    s_Swerve.drive(new Translation2d(0.1, 0.0), 0.0, false, true);

  } else if (desiredPosition.getTA() > currentTA){

    s_Swerve.drive(new Translation2d(-0.1, 0.0), 0.0, false, true);

  } else if (desiredPosition.getTA() == currentTA - Constants.Limelight.allowableError || desiredPosition.getTA() == currentY + Constants.Limelight.allowableError)

    s_Swerve.drive(new Translation2d(0.0, 0.0), 0.0, false, true);

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
