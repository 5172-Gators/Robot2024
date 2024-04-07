
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

public class NoShootSetpoint extends Command {

  Shooter s_Shooter;
  Pitch s_Pitch;
  Turret s_Turret;
  LEDs s_LEDs;

  double leftRPM;
  double rightRPM;
  double pitch;
  double yaw;
  BooleanSupplier fire;
  DoubleSupplier yaw_aim;
  DoubleSupplier pitch_aim;
  
  /** Creates a new ShootSetpoint. */
  public NoShootSetpoint(double leftRPM, double rightRPM, double pitch, double yaw, 
          DoubleSupplier yaw_aim, DoubleSupplier pitch_aim,  Pitch m_pitch, Turret m_turret, LEDs m_led) {
    this.rightRPM = rightRPM;
    this.leftRPM = leftRPM;
    this.pitch = pitch;
    this.yaw = yaw;

    this.yaw_aim = yaw_aim;
    this.pitch_aim = pitch_aim;

    s_Pitch = m_pitch;
    s_Turret = m_turret;
    s_LEDs = m_led;
    
    addRequirements(s_Shooter, s_Pitch, s_Turret, s_LEDs);
  }



// Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // SmartDashboard.putNumber("yaw sp", this.yaw);
    // SmartDashboard.putNumber("pitch sp", this.pitch);
    this.yaw += this.yaw_aim.getAsDouble()*Constants.Turret.aimCoefficient;
    // this.pitch += this.pitch_aim.getAsDouble()*0.0001;

    s_Pitch.setPositionRaw(this.pitch);
    s_Turret.setPosition(this.yaw);


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
    s_LEDs.setColor(0.99);
    s_Pitch.setPositionRaw(Constants.Pitch.intakePosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Shooter.getShooterSensor() && s_Shooter.getKickerSensor());
  }
}
