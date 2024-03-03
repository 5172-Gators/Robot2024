// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

//import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
//import com.revrobotics.SparkPIDController;

public class SetpointClimberControl extends Command {
  /** Creates a new JoystickClimberControl. */

  private final Climber s_Climber;
  private final PIDController pidController;

  private double s_Setpoint = 0.0;
  // private final double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput =0.0;
  private double s_CurrentPosition = 0.0;

  public SetpointClimberControl(Climber s_Climber, double s_Setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.pidController = new PIDController(.1, 0, 0);
    this.s_Climber = s_Climber;
    this.s_Setpoint= s_Setpoint;

    // s_CurrentPosition= s_Climber.getEncoderPosition();
    addRequirements(s_Climber);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pidController.reset();
    pidController.setSetpoint(s_Setpoint);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    pidController.setSetpoint (s_Setpoint);
    // s_CurrentPosition = s_Climber.getEncoderPosition();
    double speed = pidController.calculate(s_CurrentPosition);

    s_Climber.setSpeed(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    s_Climber.setSpeed(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
// s_CurrentPosition = s_Climber.getEncoderPosition();
    if (s_CurrentPosition <= s_Setpoint + Constants.Climber.deadband && s_CurrentPosition >= s_Setpoint - Constants.Climber.deadband) {
       s_Climber.setSpeed(0);
      return true;

    } else {
      return false;
    }
    //return false;

  }
}
