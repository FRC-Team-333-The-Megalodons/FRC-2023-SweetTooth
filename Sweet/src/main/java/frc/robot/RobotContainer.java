package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Intake.IntakeConstants;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);

    /* Drive Controls */
    private final int translationAxis = PS4Controller.Axis.kLeftY.value;
    private final int strafeAxis = PS4Controller.Axis.kLeftX.value;
    private final int rotationAxis = PS4Controller.Axis.kRightX.value;

    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kShare.value);
    private final JoystickButton lock = new JoystickButton(driver, PS4Controller.Button.kOptions.value);
    private final JoystickButton autoIntake = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    private final JoystickButton hybrid = new JoystickButton(driver, PS4Controller.Button.kCross.value);
    private final JoystickButton mid = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    private final JoystickButton fastShot = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    private final JoystickButton purple = new JoystickButton(driver, PS4Controller.Button.kTouchpad.value);
    private final JoystickButton intake = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    private final JoystickButton pivotUp = new JoystickButton(driver, PS4Controller.Button.kL3.value);
    private final JoystickButton pivotDown = new JoystickButton(driver, PS4Controller.Button.kR3.value);
    private final JoystickButton goHome = new JoystickButton(driver, PS4Controller.Button.kL2.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Intake s_Intake = new Intake();
    private final Pivot s_Pivot = new Pivot();

    /* Other Subsystems */
    private final Limelight limelight = new Limelight();

    /* Auto Chooser */
    SendableChooser<Command> s_Chooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false
            )
        );

        s_Intake.setDefaultCommand(new RunCommand(() -> s_Intake.notake(), s_Intake));

        //s_Pivot.setDefaultCommand(new GoHome(s_Pivot));
        s_Pivot.setDefaultCommand(new RunCommand(() -> s_Pivot.noPivot(), s_Pivot));

        s_Chooser.setDefaultOption("DON'T BLOODY MOVE!!!", null);
        s_Chooser.addOption("Mobility", new Mobility(s_Swerve));
        s_Chooser.addOption("Reverse! Reverse!", new ReverseMobility(s_Swerve));
        s_Chooser.addOption("Mid Shot", new EjectCube(s_Intake, IntakeConstants.hightakeSpeed).until(s_Intake::outakeAutoDone));
        s_Chooser.addOption("Charge Station", new ChargeStation(s_Swerve));
        s_Chooser.addOption("Far Charge", new FarChargeStation(s_Swerve));
        s_Chooser.addOption("Sample", new exampleAuto(s_Swerve));

        //Test Autouns
        s_Chooser.addOption("Test", new TestingAutons(s_Swerve, s_Pivot, s_Intake));

        SmartDashboard.putData(s_Chooser);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        lock.whileTrue(new RunCommand(()-> s_Swerve.lockWheels()));
        purple.whileTrue(new RunCommand(() -> s_Intake.blinkPurple(), s_Intake));
        intake.whileTrue(new IntakeCube(s_Intake));
        fastShot.whileTrue(new EjectCube(s_Intake, IntakeConstants.hightakeSpeed));
        pivotUp.whileTrue(new RunCommand(() -> s_Pivot.pivotUp(), s_Pivot));
        pivotDown.whileTrue(new RunCommand(() -> s_Pivot.pivotDown(), s_Pivot));
        autoIntake.whileTrue(new GoIntake(s_Pivot).alongWith(new IntakeCube(s_Intake)));
        mid.whileTrue(new GoMid(s_Pivot).andThen(new EjectCube(s_Intake, IntakeConstants.intakeSpeed)));
        hybrid.whileTrue(new GoHybrid(s_Pivot).andThen(new EjectCube(s_Intake, 0.15)));
        goHome.whileTrue(new GoHome(s_Pivot));
 
        /* Test Commands */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return s_Chooser.getSelected();
    }
}