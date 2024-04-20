// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimingParameters;
import frc.robot.Constants;
import frc.robot.LobTables;
import frc.robot.ShootingTables;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Shooter.NotePossession;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LEDs.LEDMode;

public class LobShot extends Command {

  private Shooter s_Shooter;
  private Pitch s_Pitch;
  private Turret s_Turret;
  private LEDs s_LEDs;
  private Kicker s_Kicker;
  private Swerve s_Swerve;

  private BooleanSupplier fire;
  private Supplier<Translation2d> translationToTargetSupplier;
  private Translation2d targetTranslation;
  private LobTables lobTables;

  private boolean inLobZone = false;
  private boolean noteInPlace = false;
  final DriverStation.Alliance alliance = DriverStation.getAlliance().get();

  /** Creates a new AutoAim. */
  public LobShot(BooleanSupplier fire, Supplier<Translation2d> translationToTargetSupplier, LobTables lobTables, 
                  Shooter m_shooter, Pitch m_pitch, Turret m_turret, Kicker m_kicker, LEDs m_led, Swerve m_swerve) {
    this.s_Shooter = m_shooter;
    this.s_Pitch = m_pitch;
    this.s_Turret = m_turret;
    this.s_LEDs = m_led;
    this.s_Kicker = m_kicker;
    this.s_Swerve = m_swerve;
    this.fire = fire;
    this.lobTables = lobTables;

    addRequirements(s_Shooter, s_Pitch, s_Turret, s_LEDs);
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
    var targetDistance = targetTranslation.getNorm();

    ChassisSpeeds V = s_Swerve.getRobotRelativeSpeeds();
    var shooterRPMSetpointAverage = (lobTables.getLeftRPM(targetDistance) + lobTables.getRightRPM(targetDistance)) / 2.0;
    var noteVelocity = Constants.Shooter.kNoteVelocityCoefficient * shooterRPMSetpointAverage; // Meters per second
    var timeOfFlight = targetDistance / noteVelocity;
    var translationOffset = new Translation2d(V.vxMetersPerSecond, V.vyMetersPerSecond).times(timeOfFlight);

    Translation2d predictedTarget = targetTranslation.minus(translationOffset);
    var predictedDistance = predictedTarget.getNorm();
    Rotation2d predictedAngle = predictedTarget.getAngle();

    AimingParameters aimingParams = lobTables.getAimingParams(predictedDistance);

    var pitch_sp = MathUtil.clamp(aimingParams.getPitchAngle(),
                           s_Pitch.encoderUnitsToDegrees(Constants.Pitch.minPitchPosition),
                           s_Pitch.encoderUnitsToDegrees(Constants.Pitch.maxPitchPosition));
    var turret_sp = predictedAngle.rotateBy(Constants.Turret.noteSpinOffset);

    s_Pitch.setPosition(Rotation2d.fromDegrees(pitch_sp));

    s_Turret.setFieldRelativeAngle(turret_sp, 
                                    s_Swerve.getPose().getRotation(),
                                    Units.degreesToRadians(s_Swerve.getAngularVelocityGyro()));

    // Check if robot is in an allowable position to lob to avoid accruing penalty points
    if (alliance == DriverStation.Alliance.Blue)
      inLobZone = s_Swerve.getPose().getX() < 10.15;
    else if (alliance == DriverStation.Alliance.Red)
      inLobZone = s_Swerve.getPose().getX() > 6.27;

    if(!noteInPlace) {
      s_LEDs.setColor(Color.kRed, LEDMode.FLASH);
      if(s_Shooter.currentNotePossession == NotePossession.HALF && !s_Shooter.shooterSensorFlag)
        s_Kicker.setKickerRPM(Constants.Kicker.kicker_creepRPM);
      if(s_Shooter.currentNotePossession == NotePossession.FULL)
        s_Kicker.setKickerRPM(-Constants.Kicker.kicker_creepRPM);
      if(s_Shooter.currentNotePossession == NotePossession.HALF && s_Shooter.shooterSensorFlag){
        s_Kicker.stopKicker(); 
        noteInPlace = true;
      }
    }

    if(noteInPlace) {
      s_Shooter.setShooterRPM(aimingParams.getShooterRPMRight(), aimingParams.getShooterRPMLeft());

      if(this.fire.getAsBoolean()) {
        s_LEDs.setFlashPeriod(0.15); // Increase flash frequency when attempting to shoot
        if(s_Shooter.shooterIsReady() && s_Turret.isReady() && s_Pitch.isReady() && inLobZone) {
          s_LEDs.setColorTimed(Color.kPurple, LEDMode.SOLID, 0.5);
          s_Kicker.setKickerRPM(Constants.Kicker.kicker_shoot);
        } else {
          s_LEDs.setColor(Color.kRed, LEDMode.FLASH);
        }
      } else {
        s_LEDs.setFlashPeriod(0.25);
        if(s_Shooter.shooterIsReady() && s_Turret.isReady() && s_Pitch.isReady() && inLobZone) {
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
    return (s_Shooter.currentNotePossession == NotePossession.NONE);
  }
}
