package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.swerve.UpdateStateEstimation;
import frc.robot.commands.turret.InitAmpScore;
import frc.robot.commands.turret.SetTurretFieldRelative;
import frc.robot.commands.turret.TeleopTurret;
import frc.robot.commands.climber.ClimbModeRoutine;
import frc.robot.commands.climber.ClimberSoftLimitOverride;
import frc.robot.commands.climber.JankyClimberPosition;
import frc.robot.commands.climber.ManualClimbControl;
import frc.robot.commands.climber.StowClimber;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.Eject;
import frc.robot.commands.intake.IntakeTravel;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.StowIntake;
import frc.robot.commands.kicker.ZeroNote;
import frc.robot.commands.led.LEDDefaultCommand;
import frc.robot.commands.limelight.HumanPlayerSignal;
import frc.robot.commands.pitch.SetPitchPosition;
import frc.robot.commands.pitch.TeleopPitch;
import frc.robot.commands.shooter.AmpScore;
import frc.robot.commands.shooter.AutoAim;
import frc.robot.commands.shooter.AutoAimShootSetpoint;
import frc.robot.commands.shooter.AutoAimWithStateEstimation;
import frc.robot.commands.shooter.LobShot;
import frc.robot.commands.shooter.NoShootSetpoint;
import frc.robot.commands.shooter.ShootSetpoint;
import frc.robot.commands.shooter.ShootSetpointCalibration;
import frc.robot.commands.shooter.ShooterCalibrationWithStateEstimation;
import frc.robot.commands.shooter.StopShooter;

import frc.robot.subsystems.*;
import frc.robot.subsystems.Limelight.CamMode;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
    /* Sendable Chooser */
    private SendableChooser<Command> autoChooser;
    
    /* Controllers */
    private final Joystick driveStick = new Joystick(0);
    private final Joystick rotateStick = new Joystick(1);
    private final Joystick operatorStick = new Joystick(2);
    
    private final Joystick testStick = new Joystick(3);

    /* Drive Controls */
    private final int translationAxis = Joystick.AxisType.kX.value;
    private final int strafeAxis = Joystick.AxisType.kY.value;
    private final int rotationAxis = Joystick.AxisType.kX.value;

    /* Driver Buttons */

    // Drive Stick
    private final JoystickButton intakeIn = new JoystickButton(driveStick, 1);
    private final JoystickButton outtake = new JoystickButton(driveStick, 2);
    private final JoystickButton beamBreakOverride = new JoystickButton(driveStick, 4);
    
    // Rotate Stick
    private final JoystickButton robotCentric = new JoystickButton(rotateStick, 1);
    private final JoystickButton zeroGyroButton = new JoystickButton(rotateStick, 2);
    private final JoystickButton shooterReverse = new JoystickButton(rotateStick, 3);

    /* Operator Controls */
    private static final int pitchAdjust = Joystick.AxisType.kY.value;
    private static final int turretRotate = Joystick.AxisType.kX.value;
    
    /* Operator Buttons */
    private final JoystickButton fireShooter = new JoystickButton(operatorStick, 1);
    private final JoystickButton stopShooter = new JoystickButton(operatorStick, 2);
    private final JoystickButton autoAim = new JoystickButton(operatorStick, 3);
    private final JoystickButton ampScore= new JoystickButton(operatorStick, 8);
    private final JoystickButton lobShotButton = new JoystickButton(operatorStick, 9);
    private final JoystickButton ampShootSetpoint = new JoystickButton(operatorStick, 10);
    private final JoystickButton shooterEject = new JoystickButton(operatorStick, 4); // poop
    // private final JoystickButton ampScoringSetpoint = new JoystickButton(operatorStick, 7);
    // private final JoystickButton testButton14 = new JoystickButton(operatorStick, 14);
    // private final JoystickButton testButton15 = new JoystickButton(operatorStick, 15);
    private final JoystickButton shooterCalibrationMode = new JoystickButton(operatorStick, 14);
    private final JoystickButton climbModeButton = new JoystickButton(operatorStick, 5);
   // private final JoystickButton shooterReverse = new JoystickButton(operatorStick, 15);
    //reverses shooter incase note gets past kicker wheels, currently assinged to thumb left (3) on rotate stick
   
   /* Test Buttons */

    // private final JoystickButton fireLobShot = new JoystickButton(testStick, 1);
    // private final JoystickButton lobShot = new JoystickButton(testStick, 2);
    private final JoystickButton testButton1 = new JoystickButton(testStick, 1);
    private final JoystickButton testButton2 = new JoystickButton(testStick, 2);
    private final JoystickButton testButton3 = new JoystickButton(testStick, 3);
    
    // private final JoystickButton setPosition = new JoystickButton(testStick, 3);

    /* Subsystems */
    public final Swerve s_Swerve;
    public final Shooter s_Shooter;
    public final Turret s_Turret;
    public final Pitch s_Pitch;
    public final Limelight s_VisionLimelight;
    public final Limelight s_DriveLimelight;
    public final Intake s_Intake;
    public final Climber s_Climber;
    public final Kicker s_Kicker;
    public final LEDs s_LEDs;

    public final ShootingTables shootingTables;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        s_Swerve = new Swerve();
        s_Shooter = new Shooter();
        s_Turret = new Turret();
        s_Pitch = new Pitch();
        s_VisionLimelight = new Limelight(CamMode.Vision);
        s_DriveLimelight = new Limelight(CamMode.Drive);
        s_Intake = new Intake();
        s_Climber = new Climber();
        s_LEDs = new LEDs();
        s_Kicker = new Kicker();
        
        shootingTables = new ShootingTables();

        autoChooser = new SendableChooser<Command>();

        /* Configure PathPlanner Commands */

        NamedCommands.registerCommand("stateEstimationShooting", new AutoAimWithStateEstimation(() -> true,
                                                                () -> s_Swerve.getTranslationToSpeaker().getNorm(), 
                                                                shootingTables,
                                                                () -> s_Swerve.getTranslationToSpeaker().getAngle().getDegrees(), 
                                                                () -> s_Swerve.getPose().getRotation().getDegrees(), 
                                                                s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs));

        NamedCommands.registerCommand("updateStateEstimation", new UpdateStateEstimation(s_Swerve, s_Turret));

        NamedCommands.registerCommand("intakeAuto", new RunIntake(s_Intake, s_Pitch, s_Turret, s_Shooter, s_Kicker, s_VisionLimelight, s_DriveLimelight, s_LEDs, () -> true));

        NamedCommands.registerCommand("shootAuto1Setpoint1", new ShootSetpoint(1800.0, 1800.0,
                                                                                    Constants.Pitch.speakerSetpoint,   
                                                                                    0.0, 
                                                                                    () -> true,
                                                                                    () -> 0, 
                                                                                    () -> 0,  
                                                                                    s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs));

        NamedCommands.registerCommand("shootAuto1Setpoint2", new ShootSetpoint(1800.0, 3000.0, 
                                                                               Constants.Pitch.stageSetpoint,  
                                                                               0.0, 
                                                                               () -> true,
                                                                               () -> 0,    
                                                                               () -> 0,     
                                                                               s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs));

        NamedCommands.registerCommand("shootAuto1Setpoint3", new ShootSetpoint(1800.0, 3000.0, 
                                                                                    0.42047, 
                                                                                    0.9, 
                                                                                    () -> true,
                                                                                    () -> 0, 
                                                                                    () -> 0, 
                                                                                    s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs));
                                    
        // NamedCommands.registerCommand("shootAutoAim", new AutoAim(() -> true, 
                                                                //    shootingTables,
                                                                //    () -> operatorStick.getX(),
                                                                //    s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs, s_VisionLimelight));
      
        NamedCommands.registerCommand("Eject", new ShootSetpoint(1500.0, 1800.0, 
                                                                      Constants.Pitch.intakePosition, 
                                                                      0, 
                                                                      () -> true,
                                                                      () -> 0, 
                                                                      () -> 0, 
                                                                      s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs));
        // NamedCommands.registerCommand("shootAuto2Setpoint1", new NoShootSetpoint(0, 0, .4862, 2.38,
     //   ()-> 0, ()-> 0, s_Pitch, s_Turret, s_LEDs));
       
        // (double leftRPM, double rightRPM, double pitch, double yaw, 
        //   DoubleSupplier yaw_aim, DoubleSupplier pitch_aim,  Pitch m_pitch, Turret m_turret, LEDs m_led) {
       
        NamedCommands.registerCommand("resetHeading", new InstantCommand(() -> s_Swerve.setHeading(s_Swerve.getHeading())));

        NamedCommands.registerCommand("setIntakeDefaultPositionDeploy", new InstantCommand(() -> s_Intake.setDefaultCommand(new DeployIntake(s_Intake))));
        NamedCommands.registerCommand("setIntakeDefaultPositionTravel", new InstantCommand(() -> s_Intake.setDefaultCommand(new IntakeTravel(s_Intake))));
        
         
         // Configure default commands
        configureDefaultCommands();

        // Configure the button bindings
        configureButtonBindings();

        // Build auto routines
        buildAutoRoutines();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureDefaultCommands(){

        /* Set Default Commands */

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driveStick.getRawAxis(strafeAxis),
                () -> -driveStick.getRawAxis(translationAxis), 
                () -> -rotateStick.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        s_Turret.setDefaultCommand(
            new TeleopTurret(
                s_Turret,
                () -> -operatorStick.getRawAxis(turretRotate) / 2 // divided by 2 to slow down the speed of rotating the turret
            )
            // new SetTurretPosition(s_Turret, Constants.Turret.R_intakingPosition)
        );

        s_Pitch.setDefaultCommand(
          
            new TeleopPitch(
                s_Pitch,
                () -> operatorStick.getY()
            )

        );

        s_Intake.setDefaultCommand(
            new IntakeTravel(s_Intake)
        );

        s_Shooter.setDefaultCommand(
            new StopShooter(s_Shooter)
        );

        s_LEDs.setDefaultCommand(
            new LEDDefaultCommand(s_LEDs, s_Shooter)
        );
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        
        /* Driver Buttons */

        zeroGyroButton.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));

        intakeIn.whileTrue(new RunIntake(s_Intake, s_Pitch, s_Turret, s_Shooter, s_Kicker, s_VisionLimelight, s_DriveLimelight, s_LEDs, beamBreakOverride.negate()));

        outtake.whileTrue(new Eject(s_Intake, s_Kicker));

        shooterReverse.whileTrue(new HumanPlayerSignal(s_VisionLimelight, s_DriveLimelight));


        /* Operator Buttons */
        autoAim.onTrue(new SequentialCommandGroup(
            new ZeroNote(s_Kicker, s_Shooter),
            new AutoAimWithStateEstimation(fireShooter, 
                                           () -> s_Swerve.getTranslationToSpeaker().getNorm(), 
                                           shootingTables,
                                           () -> s_Swerve.getTranslationToSpeaker().getAngle().getDegrees(), 
                                           () -> s_Swerve.getPose().getRotation().getDegrees(), 
                                           s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs)));

        lobShotButton.onTrue(new LobShot(fireShooter,
                                        shootingTables,
                                       () -> s_Swerve.getTranslationToSpeaker().getNorm(),
                                       () -> s_Swerve.getTranslationToSpeaker().getAngle().getDegrees(),
                                       () -> s_Swerve.getPose().getRotation().getDegrees(), 
                                       s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs));

        // shooterSetpointSpeaker.onTrue(new SequentialCommandGroup(
        //     new ZeroNote(s_Kicker, s_Shooter),
        //     new ShootSetpoint(1800.0, 1800.0, 
        //                       Constants.Pitch.speakerSetpoint, 
        //                       0.0, 
        //                       fireShooter,
        //                       () -> operatorStick.getX(), 
        //                       () -> operatorStick.getY(),
        //                       s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs)));

        //shooterSetpointAmp.onTrue(new ShootSetpoint(850.0, 850.0, Constants.Pitch.ampSetpoint, Constants.Turret.ampTurretSetpoint, fireShooter,
        //                             s_Shooter, s_Pitch, s_Turret, s_LEDs));
        //USE THIS #10
        ampShootSetpoint.onTrue(new ShootSetpoint(850.0, 850.0, 
                                                  Constants.Pitch.setpointAmp, 
                                                  -12.047677, 
                                                  fireShooter, 
                                                  () -> operatorStick.getX(), 
                                                  () -> operatorStick.getY(),
                                                  s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs));

        shooterReverse.onTrue(new ShootSetpoint(-500.0, -500.0, 
                                                Constants.Pitch.intakePosition, 
                                                0, 
                                                fireShooter,
                                                () -> operatorStick.getX(), 
                                                () -> operatorStick.getY(), 
                                                s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs));

        shooterEject.onTrue(new SequentialCommandGroup(
            new ZeroNote(s_Kicker, s_Shooter),
            new ShootSetpoint(500.0, 500.0, 
                              Constants.Pitch.intakePosition, 
                              0, 
                              fireShooter,
                              () -> operatorStick.getX(), 
                              () -> operatorStick.getY(), 
                              s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs)));
                        
        // ampScoringSetpoint.onTrue(new AmpScore(900.0, 900.0, 
        //                                        0.500488, 
        //                                        fireShooter, 
        //                                        () -> operatorStick.getY(), 
        //                                        s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs, false));

        stopShooter.onTrue(new StopShooter(s_Shooter));

        climbModeButton.whileTrue(new ManualClimbControl(() -> operatorStick.getY(), s_Climber, s_Pitch, s_Turret)); // Climb Mode Routine

        SmartDashboard.putData("ClimberOverride", new ClimberSoftLimitOverride(() -> operatorStick.getY(), s_Climber, s_Pitch, s_Turret));

        ampScore.onTrue(new AmpScore(1500.0, 1500.0,
                                        fireShooter,
                                        () -> operatorStick.getX(),
                                        () -> operatorStick.getY(),
                                        s_Shooter, s_Pitch, s_Turret, s_Climber, s_Kicker, s_LEDs));

        /* Test Buttons */

        // lobShot.onTrue(new ShootSetpoint(5000, 5000, Constants.Pitch.lobSetPoint, 2.04, fireLobShot, () -> operatorStick.getY(), () -> operatorStick.getX(),
        //                                     s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs));

        shooterCalibrationMode.onTrue(new ShooterCalibrationWithStateEstimation(1700.0, 1700.0, 
                                                                                .64, 
                                                                                0, 
                                                                                fireShooter,
                                                                                () -> s_Swerve.getTranslationToSpeaker().getAngle().getDegrees(), 
                                                                                () -> s_Swerve.getPose().getRotation().getDegrees(), 
                                                                                () -> operatorStick.getY(), 
                                                                                s_Shooter, s_Pitch, s_Turret, s_Kicker, s_LEDs, s_VisionLimelight,
                                                                                new JoystickButton(operatorStick, 12),
                                                                                new JoystickButton(operatorStick, 15),
                                                                                new JoystickButton(operatorStick, 11),
                                                                                new JoystickButton(operatorStick, 16)
                                                                                 ));
        
        // testButton1.whileTrue(new SetPitchPosition(s_Pitch, 0.75));
        // testButton1.whileTrue(new SetPitchPosition(s_Pitch, 0.75));
        // testButton1.whileTrue(new SetPitchPosition(s_Pitch, 0.75));
        // SmartDashboard.putData("Test Button 1", new SetPitchPosition(s_Pitch, 0.75));
        // SmartDashboard.putData("Test Button 2", new SetPitchPosition(s_Pitch, 1.0));
        // SmartDashboard.putData("Test Button 3", new SetPitchPosition(s_Pitch, 1.25));

        // new JoystickButton(operatorStick, 15).whileTrue(new SetTurretFieldRelative(s_Turret, Rotation2d.fromDegrees(90), () -> s_Swerve.getPose().getRotation().getDegrees()));
        


    }

    private void buildAutoRoutines() {

        autoChooser.addOption("driveOnlyAuto", new PathPlannerAuto("driveOnlyAuto"));
        autoChooser.addOption("auto1", new PathPlannerAuto("auto1"));  
        autoChooser.addOption("StateEstimation", new PathPlannerAuto("StateEstimation"));
        // autoChooser.addOption("shootParkSource", new PathPlannerAuto("shootParkSource"));
        // autoChooser.addOption("shootParkSourceAuto", new PathPlannerAuto("shootParkSourceAuto"));
        // autoChooser.addOption("auto2", new PathPlannerAuto("auto2"));
        // autoChooser.addOption("testAuto", new PathPlannerAuto("testAuto"));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
         
        return autoChooser.getSelected();
    }
}
