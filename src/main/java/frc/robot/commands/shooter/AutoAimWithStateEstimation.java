// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimingParameters;
import frc.robot.AimingTolerances;
import frc.robot.Constants;
import frc.robot.ShooterInterpolatingDoubleTable;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.subsystems.Shooter.NotePossession;

public class AutoAimWithStateEstimation extends Command {

  private Shooter s_Shooter;
  private Pitch s_Pitch;
  private Turret s_Turret;
  private LEDs s_LEDs;
  private Kicker s_Kicker;
  private Swerve s_Swerve;

  private BooleanSupplier fire;
  private BooleanSupplier ceaseFire;
  private BooleanSupplier forceFire;
  private Supplier<Translation2d> translationToTargetSupplier;
  private Translation2d targetTranslation;
  private ShooterInterpolatingDoubleTable tbl;
  private AimingTolerances tolerances;

  private boolean noteInPlace = false;

  /** Creates a new AutoAim. */
  public AutoAimWithStateEstimation(BooleanSupplier fire, Supplier<Translation2d> translationToTargetSupplier, ShooterInterpolatingDoubleTable tbl,
                  AimingTolerances tolerances, BooleanSupplier ceaseFire, BooleanSupplier forceFire, 
                  Shooter m_shooter, Pitch m_pitch, Turret m_turret, Kicker m_kicker, LEDs m_led, Swerve m_swerve) {
    this.s_Shooter = m_shooter;
    this.s_Pitch = m_pitch;
    this.s_Turret = m_turret;
    this.s_LEDs = m_led;
    this.s_Kicker = m_kicker;
    this.fire = fire;
    this.translationToTargetSupplier = translationToTargetSupplier;
    this.tbl = tbl;
    this.s_Swerve = m_swerve;
    this.tolerances = tolerances;
    this.ceaseFire = ceaseFire;
    this.forceFire = forceFire;

    addRequirements(s_Shooter, s_Pitch, s_Turret, s_LEDs, s_Kicker);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteInPlace = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Motion compensation
    targetTranslation = translationToTargetSupplier.get();
    var targetDistance = targetTranslation.getNorm(); // floor distance, meters

    ChassisSpeeds V = s_Swerve.getRobotRelativeSpeeds();
    var shooterRPMSetpointAverage = (tbl.getLeftRPM(targetDistance) + tbl.getRightRPM(targetDistance)) / 2.0;
    var noteVelocity = Constants.Targeting.kNoteVelocityCoefficient * shooterRPMSetpointAverage; // Meters per second
    var targetDist3D = Math.sqrt(Math.pow(targetDistance,2) + Math.pow(Constants.Field.speakerHeightMeters,2));
    var timeOfFlight = targetDist3D / noteVelocity;
    var translationOffset = new Translation2d(V.vxMetersPerSecond, V.vyMetersPerSecond).times(timeOfFlight);

    Translation2d predictedTarget = targetTranslation.minus(translationOffset);
    var predictedDistance = predictedTarget.getNorm();
    Rotation2d predictedAngle = predictedTarget.getAngle();

    AimingParameters aimingParams = tbl.getAimingParams(predictedDistance);

    var pitch_sp = MathUtil.clamp(tbl.getPitch(predictedDistance),
                           s_Pitch.encoderUnitsToDegrees(Constants.Pitch.minPitchPosition),
                           s_Pitch.encoderUnitsToDegrees(Constants.Pitch.maxPitchPosition));
    var turret_sp = predictedAngle.rotateBy(Constants.Turret.noteSpinOffset);

    // Calculate angular velocity (rad/s) of robot to predicted target translation for turret FF
    var dTheta = (predictedTarget.getX()*V.vyMetersPerSecond - predictedTarget.getY()*V.vxMetersPerSecond) /
                  Math.pow(predictedTarget.getNorm(),2);

    // Calculate normal velocity (m/s) of robot to predicted target for flywheel FF
    var dT = (predictedTarget.getX()*V.vxMetersPerSecond + predictedTarget.getY()*V.vyMetersPerSecond) / 
              predictedTarget.getNorm();

    var dToriginal = (targetTranslation.getX()*V.vxMetersPerSecond+targetTranslation.getY()*V.vyMetersPerSecond)/
                      targetTranslation.getNorm();
    // Calculate angular velocity of pitch (rad/s) of robot to predicted target for pitch FF
    var dPhi = Constants.Field.speakerHeightMeters * dT / 
                (Math.pow(predictedTarget.getNorm(),2) + 
                Math.pow(Constants.Field.speakerHeightMeters,2));

    var turretFF = dTheta * Constants.Targeting.kTargeting_dTheta_FF;
    var shooterFF = Math.max(dT * Constants.Targeting.kTargeting_dT_FF, 0);
    var pitchFF = dPhi * Constants.Targeting.kTargeting_dPhi_FF;

    SmartDashboard.putNumber("Right Desired RPM", aimingParams.getShooterRPMRight());
    SmartDashboard.putNumber("Left Desired RPM", aimingParams.getShooterRPMLeft());
    SmartDashboard.putNumber("Pitch Setpoint", aimingParams.getPitchAngle());
    SmartDashboard.putNumber("pitch sp", pitch_sp);
    // SmartDashboard.putNumber("turret FF", turretFF);
    SmartDashboard.putNumber("pitch FF", pitchFF);
    SmartDashboard.putNumber("d theta", dTheta);
    SmartDashboard.putNumber("dPhi", dPhi);
    SmartDashboard.putNumber("dT", dT);
   
    s_Pitch.setPosition(Rotation2d.fromDegrees(pitch_sp), pitchFF);

    s_Turret.setFieldRelativeAngle(turret_sp, 
                                    s_Swerve.getPose().getRotation(),
                                    Units.degreesToRadians(s_Swerve.getAngularVelocityGyro()),
                                    turretFF);
  
    if(!noteInPlace) {
      s_LEDs.setColor(Color.kRed, LEDMode.FLASH);
      if(s_Shooter.currentNotePossession == NotePossession.HALF && !s_Shooter.shooterSensorFlag)
        s_Kicker.setKickerRPM(Constants.Kicker.kicker_creepRPM);
      if(s_Shooter.currentNotePossession == NotePossession.FULL)
        s_Kicker.setKickerRPM(-Constants.Kicker.kicker_creepReverse);
      if(s_Shooter.currentNotePossession == NotePossession.HALF && s_Shooter.shooterSensorFlag) {
        s_Kicker.stopKicker(); 
        noteInPlace = true;
      }
    }

    if(noteInPlace) {
      s_Shooter.setShooterRPM(aimingParams.getShooterRPMRight(), aimingParams.getShooterRPMLeft(), shooterFF);

      if(this.fire.getAsBoolean()) {
        s_LEDs.setFlashPeriod(0.15); // Increase flash frequency when attempting to shoot
        if(s_Shooter.isReady(tolerances.leftTol, tolerances.rightTol) && s_Turret.isReady(tolerances.turretTol) && s_Pitch.isReady(tolerances.pitchTol) && !ceaseFire.getAsBoolean() || forceFire.getAsBoolean()) {
          s_LEDs.setColorTimed(Color.kPurple, LEDMode.SOLID, 0.5);
          s_Kicker.setKickerRPM(Constants.Kicker.kicker_shoot);
        } else {
          s_LEDs.setColor(Color.kRed, LEDMode.FLASH);
        }
      } else {
        s_LEDs.setFlashPeriod(0.25);
        if(s_Shooter.isReady(tolerances.leftTol, tolerances.rightTol) && s_Turret.isReady(tolerances.turretTol) && s_Pitch.isReady(tolerances.pitchTol) && !ceaseFire.getAsBoolean() | forceFire.getAsBoolean()) {
          s_LEDs.setColor(Color.kPurple);
          s_Kicker.stopKicker();
        } else {
          s_LEDs.setColor(Color.kRed, LEDMode.FLASH);
          s_Kicker.stopKicker();
        }
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_LEDs.resetFlashPeriodToDefault(); // Reset flash period to default
    s_Shooter.setShooterRPM(0, 0);
    s_Kicker.stopKicker();
    s_LEDs.setColor(Color.kBlack);
    s_Pitch.setPositionRaw(Constants.Pitch.intakePosition);
    s_Turret.setPosition(Constants.Turret.R_intakingPosition);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Shooter.currentNotePossession == NotePossession.NONE;
  }
}
