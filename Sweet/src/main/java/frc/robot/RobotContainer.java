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
import frc.robot.subsystems.*;
import frc.robot.commands.ManualIntake;
import frc.robot.commands.GoIntakePosition;
import frc.robot.commands.ManualOutake;
import frc.robot.commands.ManualPivotDown;
import frc.robot.commands.ManualPivotUp;
import frc.robot.commands.TeleopSwerve;
import frc.robot.autos.ComeBackScoreAuto;
import frc.robot.autos.Mobility;
import frc.robot.autos.ScoreMobility;
import frc.robot.autos.ShootMobility;
import frc.robot.autos.BrokenDownAutos.ComeBackMobility;
import frc.robot.autos.BrokenDownAutos.ComeOutAndBAckMobility;
import frc.robot.autos.BrokenDownAutos.MibilityBack;
import frc.robot.autos.BrokenDownAutos.scoreHybrid;
import frc.robot.commands.GoHomePosition;
import frc.robot.commands.GoScoreHybrid;
import frc.robot.commands.GoIntakeFloor;
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
    public final JoystickButton goIntakeFloor = new JoystickButton(driver, PS4Controller.Button.kTouchpad.value);
    
    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Pivot s_Pivot = new Pivot();
    private final Intake s_Intake = new Intake();
    SendableChooser<Command> s_Chooser = new SendableChooser<>();
    SendableChooser<Command> p_Chooser;
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

        s_Intake.setDefaultCommand(new RunCommand(() -> s_Intake.outake(), s_Intake));
            s_Pivot.setDefaultCommand(new RunCommand(() -> s_Pivot.noPivot(), s_Pivot));
            
            
            s_Chooser.setDefaultOption("NO_AUTON", null);
            s_Chooser.addOption("COME_BACK_MOBILITY", new ComeBackMobility(s_Swerve));
            s_Chooser.addOption("IN_OUT_MOBILITY", new ComeOutAndBAckMobility(s_Swerve));
            s_Chooser.addOption("BACK_MOBILITY", new MibilityBack(s_Swerve));
            s_Chooser.addOption("SCORE_HYBRID", new scoreHybrid(s_Swerve, s_Intake, s_Pivot));
            s_Chooser.addOption("COME_BACK_SCORE", new ComeBackScoreAuto(s_Swerve, s_Pivot, s_Intake));
            s_Chooser.addOption("MOBILITY", new Mobility(s_Swerve));
            s_Chooser.addOption("SCORE_MOBILITY", new ScoreMobility(s_Intake, s_Pivot, s_Swerve));
            s_Chooser.addOption("SHOOT_MOBILITY", new ShootMobility(s_Swerve, s_Intake));
            
            SmartDashboard.putData(s_Chooser);
            configureButtonBindings();
    
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
        pivotButUp.whileTrue(new ManualPivotUp(s_Pivot));
        pivotButDown.whileTrue(new ManualPivotDown(s_Pivot));
        intakeButIn.whileTrue(new ManualIntake(s_Intake));
        intakButeOut.whileTrue(new ManualOutake(s_Intake));
        goHome.whileTrue(new GoHomePosition (s_Pivot));
        goIntake.whileTrue(new GoIntakePosition(s_Pivot));
        goScoreHybrid.whileTrue(new GoScoreHybrid(s_Pivot, s_Intake));
        goIntakeFloor.whileTrue(new GoIntakeFloor(s_Pivot, s_Intake));
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