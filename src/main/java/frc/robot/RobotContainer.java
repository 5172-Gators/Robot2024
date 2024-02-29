package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.auto.AutoBuilder;


import frc.robot.commands.swerve.TeleopSwerve;
// import frc.robot.commands.trap.StopArm;
import frc.robot.commands.turret.TeleopTurret;
import frc.robot.commands.turret.TeleopPitch;
import frc.robot.commands.climber.JoystickClimberControl;
import frc.robot.commands.intake.MoveJoint;
import frc.robot.commands.intake.DeployIntake;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.SetIntakeWheels;
import frc.robot.commands.intake.StopIntake;
import frc.robot.commands.intake.StowIntake;
import frc.robot.commands.shooter.SetKicker;
import frc.robot.commands.shooter.TeleopShoot;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pitch;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Limelight;
// import frc.robot.subsystems.TrapScore;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    
    /* Sendable Chooser */
    private SendableChooser<Command> chooser = new SendableChooser<>();
    
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
    private final JoystickButton startShooter = new JoystickButton(operatorStick, 1);

    // private final JoystickButton trapScore = new JoystickButton(operatorStick, 6);
    // private final JoystickButton deployTrapScore = new JoystickButton(operatorStick, 7);
    // private final JoystickButton toggleClimberJoystickControl = new JoystickButton(operatorStick, 8); // will be a onTrue, then onFalse will stop climber control
    

    // private final JoystickButton alignLimelight = new JoystickButton(operatorStick, 4);

    private final JoystickButton testSetPosition = new JoystickButton(testStick, 1);

    /* Test Buttons */
    
    // private final JoystickButton setPosition = new JoystickButton(testStick, 3);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Shooter s_Shooter = new Shooter();
    private final Turret s_Turret = new Turret();
    private final Pitch s_Pitch = new Pitch();
    private final Limelight s_Limelight = new Limelight();
    private final Intake s_Intake = new Intake();
    private final Climber s_Climber = new Climber();
    // private final TrapScore s_TrapScore = new TrapScore();
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        
        // chooser = AutoBuilder.buildAutoChooser();

        // // Define Autos + Configure Sendable Chooser
        // chooser.addOption("Deploy Test Auto", new PathPlannerAuto("deployTestAuto"));
        // chooser.addOption("Test Auto", new PathPlannerAuto("testAuto"));
        // chooser.addOption("Nothing", null);
        // SmartDashboard.putData("AutoChooser", chooser);
        // SmartDashboard.putNumber("SpeedLimit", 1);
       
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

    //    s_Turret.setDefaultCommand(
    //        new TeleopTurret(
    //         s_Turret,
    //         () -> operatorStick.getRawAxis(turretRotate) / 2 // divided by 2 to slow down the speed of rotating the turret
    //        )
    //    );

        s_Pitch.setDefaultCommand(
           new TeleopPitch(
            s_Pitch,
            () -> -operatorStick.getRawAxis(pitchAdjust)
           )
       );

        s_Climber.setDefaultCommand(
            new JoystickClimberControl(
             s_Climber,
             () -> -testStick.getRawAxis(pitchAdjust) // y-axis           
            )

        );

        // s_TrapScore.setDefaultCommand(
        //    new StopArm(
        //     s_TrapScore
        //    )
        // );

        // s_Intake.setDefaultCommand(
        //     new MoveJoint(
        //         s_Intake,
        //         () -> testStick.getRawAxis(pitchAdjust)
        //     )
        // );


        // Configure the button bindings
        configureButtonBindings();
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

        deployIntake.onTrue(new IntakeCommand(s_Intake, s_Shooter));

        stowIntake.onTrue(new StopIntake(s_Shooter, s_Intake));

        justDeploy.onTrue(new DeployIntake(s_Intake));


        /* Operator Buttons */
        
        startShooter.onTrue(new TeleopShoot(s_Shooter));

        stopShooter.onTrue(new InstantCommand(() -> s_Shooter.stopShooter()));

        /* Test Buttons */

        // testSetPosition.onTrue(new SetPitchPosition(s_Pitch, 0));


    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new PathPlannerAuto("exampleAuto");
    }
}
