// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Shooter;

public class TeleopStartShooterPID extends Command {
  /** Creates a new shoot. */

  Shooter s_Shooter;
double s_Speed;
  double rightRatio;
  double leftRatio;
  double kickerSpeed;

  boolean kickerSensor;
  boolean kickerSensor2;

  
  // speed of the motor will always be 1

  public TeleopStartShooterPID(Shooter s_ShooterPID, double s_Speed, double leftRatio, double rightRatio) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_ShooterPID = s_ShooterPID;
    this.rightRatio = rightRatio;
    this.leftRatio = leftRatio;
    this.s_Speed = s_Speed;

    addRequirements(s_Shooter);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    // sets the shooter speed
    s_ShooterPID.setShooter(s_Speed, leftRatio, rightRatio);



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_ShooterPID.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (s_ShooterPID.getRightSpeed() >= s_Speed*rightRatio-shooterDeadband && s_ShooterPID.getLeftSpeed()>= s_Speed*leftRatio-shooterDeadband){
      return true;
    }else{
      return false;
    }
  
    
  }
}
