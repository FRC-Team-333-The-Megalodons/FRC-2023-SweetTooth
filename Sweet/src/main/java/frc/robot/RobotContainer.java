package frc.robot;

import com.fasterxml.jackson.databind.ser.std.StdKeySerializers.Default;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.IntakeIn;
import frc.robot.commands.IntakeOut;
import frc.robot.commands.PivotDown;
import frc.robot.commands.PivotUp;
import frc.robot.commands.TeleopSwerve;
import frc.robot.autos.comeBackMobility;
import frc.robot.autos.inAndOut;
import frc.robot.autos.inAndOutScore;
import frc.robot.autos.justScore;
import frc.robot.autos.mobility;
import frc.robot.autos.reverse;
import frc.robot.autos.scoreHybrid;
import frc.robot.autos.scoreMobility;
import frc.robot.commands.GoHome;
import frc.robot.commands.GoIntake;
import frc.robot.commands.GoScoreHybrid;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;


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

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS4Controller.Button.kShare.value);
    
    private final JoystickButton pivotButUp = new JoystickButton(driver, PS4Controller.Button.kTriangle.value);
    public final JoystickButton pivotButDown = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
    public final JoystickButton intakeButIn = new JoystickButton(driver, PS4Controller.Button.kR1.value);
    public final JoystickButton intakButeOut = new JoystickButton(driver, PS4Controller.Button.kL1.value);
    public final JoystickButton goHome = new JoystickButton(driver, PS4Controller.Button.kR2.value);
    public final JoystickButton goIntake = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    public final JoystickButton goScoreHybrid = new JoystickButton(driver, PS4Controller.Button.kSquare.value);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot s_Pivot = new Pivot();
    private final Intake s_Intake = new Intake();
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureButtonBindings();

         s_Pivot.setDefaultCommand(new RunCommand(() -> s_Pivot.noPivot(), s_Pivot));
         s_Intake.setDefaultCommand(new RunCommand(() -> s_Intake.noIntake(), s_Intake));

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false
            )
        );
    }


    /**
     * Use this method to define your button->command mappings. Buttons can be created by[]
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link PS4Controller}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons  and their actions*/
       zeroGyro.onTrue(new InstantCommand(()   -> s_Swerve.zeroGyro()));
        pivotButUp.whileTrue(new PivotUp(s_Pivot));
        pivotButDown.whileTrue(new PivotDown(s_Pivot));
        intakeButIn.whileTrue(new IntakeIn(s_Intake));
        intakButeOut.whileTrue(new IntakeOut(s_Intake));
        goHome.whileTrue(new GoHome (s_Pivot));
        goIntake.whileTrue(new GoIntake(s_Pivot));
        goScoreHybrid.whileTrue(new GoScoreHybrid(s_Pivot, s_Intake));
    }

    // smart dashboard stuff 
    // inseart here 
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        /*switch (selectedAuto) {
        case Robot.kNoAuto: return null;
        case Robot.kMobility: return mobility(s_Swerve);
        case Robot.justScore: return justScore(s_Intake);
        case Robot.scoreMobility: return scoreMobility(s_Intake, s_Pivot, s_Swerve);
*/
        // An ExampleCommand will run in autonomous
        //return new justScore(s_Intake);
        //return new scoreMobility(s_Intake, s_Pivot, s_Swerve);
        // return new reverse(s_Swerve);
        //return new mobiility(s_Swerve)
        //return null;
        // return new scoreHybrid(s_Pivot, s_Intake);
        //return new comeBackMobility(s_Swerve);
        return new inAndOut(s_Swerve);
        //return new inAndOutScore(s_Swerve, s_Pivot, s_Intake);
    }
}