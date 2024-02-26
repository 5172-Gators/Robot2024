package frc.robot.commands.turret;

import java.lang.Math;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Pitch;

/* Rotate Command for Turret During Teleop */
public class TeleopPitch extends Command {
  /** Creates a new TeleopTurret. */
  private Pitch s_Pitch;
  private DoubleSupplier m_joystickPosition;

  public TeleopPitch (Pitch s_Pitch, DoubleSupplier m_joystickPosition) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Pitch = s_Pitch;
    addRequirements(s_Pitch);
    
    this.m_joystickPosition = m_joystickPosition;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
   public void execute() {
    // translates the position of a joystick into a value that the motor can accept
    double j_Position= m_joystickPosition.getAsDouble();
   
    // runs the motor based on the position of the joystick
    s_Pitch.joystickPitchControl(j_Position);
        
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}

