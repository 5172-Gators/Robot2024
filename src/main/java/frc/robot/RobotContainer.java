package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import frc.robot.commands.swerve.TeleopSwerve;
import frc.robot.commands.turret.SetPitchPosition;
import frc.robot.commands.turret.SetTurretPosition;
import frc.robot.commands.turret.TeleopPitch;
import frc.robot.commands.turret.TeleopTurret;
import frc.robot.commands.climber.JoystickClimberControl;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.Eject;
import frc.robot.commands.intake.IntakeTravel;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.StowIntake;
import frc.robot.commands.led.LEDTest;
import frc.robot.commands.shooter.ShootSetpoint;
import frc.robot.commands.shooter.StopShooter;

import frc.robot.subsystems.*;

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
    private final JoystickButton deployIntake = new JoystickButton(driveStick, 1);
    private final JoystickButton stowIntake = new JoystickButton(driveStick, 4);
    private final JoystickButton justDeploy = new JoystickButton(driveStick, 2);
    private final JoystickButton robotCentric = new JoystickButton(driveStick, 12);
    
   
    // Rotate Stick
    private final JoystickButton zeroGyroButton = new JoystickButton(rotateStick, 2);
    

    /* Operator Controls */
    private static final int pitchAdjust = Joystick.AxisType.kY.value;
    private static final int turretRotate = Joystick.AxisType.kX.value;
    
    /* Operator Buttons */
    private final JoystickButton stopShooter = new JoystickButton(operatorStick, 2);
    private final JoystickButton fireShooter = new JoystickButton(operatorStick, 1);
    private final JoystickButton testSetPosition = new JoystickButton(operatorStick, 3);
    private final JoystickButton shooterSetpoint1 = new JoystickButton(operatorStick, 8);
    private final JoystickButton shooterSetpoint2 = new JoystickButton(operatorStick, 9);
    private final JoystickButton shooterSetpoint3 = new JoystickButton(operatorStick, 10);

    private final JoystickButton testButton14 = new JoystickButton(operatorStick, 14);
    private final JoystickButton testButton15 = new JoystickButton(operatorStick, 15);
    private final JoystickButton testButton16 = new JoystickButton(operatorStick, 16);


    // private final JoystickButton trapScore = new JoystickButton(operatorStick, 6);
    // private final JoystickButton deployTrapScore = new JoystickButton(operatorStick, 7);
    // private final JoystickButton toggleClimberJoystickControl = new JoystickButton(operatorStick, 8); // will be a onTrue, then onFalse will stop climber control
    

    // private final JoystickButton alignLimelight = new JoystickButton(operatorStick, 4);

    // private final JoystickButton testSetPosition = new JoystickButton(testStick, 1);
    private final JoystickButton testAprilTag = new JoystickButton(testStick, 1);

    /* Test Buttons */
    
    // private final JoystickButton setPosition = new JoystickButton(testStick, 3);

    /* Subsystems */
    private final Swerve s_Swerve;
    private final Shooter s_Shooter;
    private final Turret s_Turret;
    private final Pitch s_Pitch;
    private final Limelight s_Limelight;
    private final Intake s_Intake;
    private final Climber s_Climber;
    private final LEDs s_LEDs;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        s_Swerve = new Swerve();
        s_Shooter = new Shooter();
        s_Turret = new Turret();
        s_Pitch = new Pitch();
        s_Limelight = new Limelight();
        s_Intake = new Intake();
        s_Climber = new Climber();
        s_LEDs = new LEDs();

        autoChooser = new SendableChooser<Command>();

        /* Configure PathPlanner Commands */
        NamedCommands.registerCommand("intakeAuto", new RunIntake(s_Intake, s_Pitch, s_Turret, s_Shooter));
        NamedCommands.registerCommand("shootAuto1Setpoint1", new ShootSetpoint(1800.0, 1800.0,Constants.Pitch.speakerSetpoint, 0.0, () -> true,
                                    () -> 0, () -> 0, s_Shooter, s_Pitch, s_Turret));
        NamedCommands.registerCommand("shootAuto1Setpoint2", new ShootSetpoint(1800.0, 3000.0, Constants.Pitch.stageSetpoint, 0.0, () -> true,
                                    () -> 0, () -> 0, s_Shooter, s_Pitch, s_Turret));
        NamedCommands.registerCommand("shootAuto1Setpoint3", new ShootSetpoint(1800.0, 3000.0, 0.42047, 0.9, () -> true,
                                    () -> 0, () -> 0, s_Shooter, s_Pitch, s_Turret));
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
            () -> operatorStick.getRawAxis(turretRotate) / 2 // divided by 2 to slow down the speed of rotating the turret
           )
            // new SetTurretPosition(s_Turret, Constants.Turret.R_intakingPosition)
       );

        s_Pitch.setDefaultCommand(
           new TeleopPitch(
            s_Pitch,
            () -> -operatorStick.getRawAxis(pitchAdjust)
           )
            // new SetPitchPosition(s_Pitch, Constants.Pitch.P_intakingPosition)
       );

        s_Climber.setDefaultCommand(
            new JoystickClimberControl(
             s_Climber,
             () -> -testStick.getRawAxis(pitchAdjust) // y-axis           
            )

        );

        s_Intake.setDefaultCommand(
            new IntakeTravel(s_Intake)
        );

        s_Shooter.setDefaultCommand(
            new StopShooter(s_Shooter)
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

        deployIntake.whileTrue(new RunIntake(s_Intake, s_Pitch, s_Turret, s_Shooter));

        stowIntake.onTrue(new StowIntake(s_Intake));

        justDeploy.whileTrue(new Eject(s_Intake,s_Shooter));


        /* Operator Buttons */
        
        // startShooter.whileTrue(new Shoot(s_Shooter));

        shooterSetpoint1.onTrue(new ShootSetpoint(1800.0, 3000.0, Constants.Pitch.stageSetpoint, 1.85714, fireShooter, 
                                    () -> operatorStick.getX(), () -> operatorStick.getY(), s_Shooter, s_Pitch, s_Turret));

        shooterSetpoint2.onTrue(new ShootSetpoint(1800.0, 1800.0, Constants.Pitch.speakerSetpoint, 0.0, fireShooter,
                                    () -> operatorStick.getX(), () -> operatorStick.getY(), s_Shooter, s_Pitch, s_Turret));

        shooterSetpoint3.onTrue(new ShootSetpoint(1800.0, 3000.0, Constants.Pitch.ampSetpoint, -2.142856597, fireShooter,
                                    () -> operatorStick.getX(), () -> operatorStick.getY(), s_Shooter, s_Pitch, s_Turret));

        stopShooter.onTrue(new StopShooter(s_Shooter));

        // testSetPosition.whileTrue(new DeployIntake(s_Intake));

        // testSetPosition.whileTrue(new InstantCommand(() -> s_Shooter.setShooter(2750, 2000))).onFalse(new InstantCommand(() -> s_Shooter.stopShooter()));

        // testSetPosition.whileTrue(new InstantCommand(() -> s_Intake.setIntakeRPM(3000))).onFalse(new InstantCommand(() -> s_Intake.stopIntake()));

        // testSetPosition.whileTrue(new InstantCommand(() -> s_Shooter.setKickerRPM(500))).onFalse(new InstantCommand(() -> s_Shooter.stopKicker()));

        testButton14.whileTrue(new SetTurretPosition(s_Turret, 0));
        testButton15.whileTrue(new SetTurretPosition(s_Turret, 1.64));
        testButton16.whileTrue(new SetTurretPosition(s_Turret, -2.46));

        /* Test Buttons */

        testAprilTag.onTrue(new LEDTest(s_LEDs, Constants.Colors.green));


    }

    private void buildAutoRoutines(){

    //   AutoBuilder.buildAuto("forward4intake");
    //   AutoBuilder.buildAuto("auto1");

        autoChooser.addOption("forward4intake", new PathPlannerAuto("forward4intake"));
        autoChooser.addOption("auto1", new PathPlannerAuto("auto1"));
        PathPlannerAuto auto1 = new PathPlannerAuto("auto1");
        PathPlannerAuto.getPathGroupFromAutoFile("auto1").get(1).getPoint(10).position.getY();    
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
