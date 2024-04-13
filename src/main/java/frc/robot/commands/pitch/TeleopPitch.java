package frc.robot.commands.pitch;

import java.lang.Math;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pitch;

/* Rotate Command for Turret During Teleop */
public class TeleopPitch extends Command {
  /** Creates a new TeleopTurret. */
  private Pitch s_Pitch;
  private DoubleSupplier joy;
  private double holdSetpoint;

  public TeleopPitch (Pitch s_Pitch, DoubleSupplier s_pitchControl) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Pitch = s_Pitch;
    this.joy = s_pitchControl;
    addRequirements(s_Pitch);
  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
    // translates the position of a joystick into a value that the motor can accept
    // setpoint = MathUtil.clamp(setpoint + m_joystickPosition.getAsDouble()*Constants.Pitch.teleopControlInputCoefficient,
    //                         Constants.Pitch.minPitchPosition,
    //                         Constants.Pitch.maxPitchPosition);
    
    // double dbJoy = MathUtil.applyDeadband(-joy.getAsDouble(), 0.1); 
    
    if (Math.abs(joy.getAsDouble()) > 0) {
      if(joy.getAsDouble() < 0 && s_Pitch.getRawPitchPosition() < Constants.Pitch.minPitchPosition + 0.1) {
        s_Pitch.setPositionRaw(Constants.Pitch.minPitchPosition);
      } else if(joy.getAsDouble() > 0 && s_Pitch.getRawPitchPosition() > Constants.Pitch.maxPitchPosition - 0.1) {
        s_Pitch.setPositionRaw(Constants.Pitch.maxPitchPosition);
      } else {
        s_Pitch.joystickPitchControl(joy.getAsDouble()*Constants.Pitch.teleopControlInputCoefficient);
      }

    } else {
      holdSetpoint = MathUtil.clamp(s_Pitch.getRawPitchPosition(), 
                                Constants.Pitch.minPitchPosition,
                                Constants.Pitch.maxPitchPosition);
      s_Pitch.setPositionRaw(holdSetpoint);
    }
    // Sets motor position based on joystick input
    // s_Pitch.setPositionRaw(setpoint);
        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // s_Pitch.movePitch(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

