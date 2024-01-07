// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeConstants;
import frc.robot.commands.EjectCube;
import frc.robot.commands.GoHome;
import frc.robot.commands.GoIntake;
import frc.robot.commands.IntakeCube;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import java.util.List;

public class TestingAutons extends SequentialCommandGroup {
  /** Creates a new TestingAutons. */
  public TestingAutons(Swerve swerve, Pivot pivot, Intake intake) {
    TrajectoryConfig config =
        new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.SwerveDrive.swerveKinematics)
            .setReversed(true);

    Trajectory trajectory =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5, 0, new Rotation2d(0)),
            List.of(new Translation2d(3, 0), new Translation2d(1, 0)),
            new Pose2d(0, 0, new Rotation2d(-2.7)),
            config);

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            Constants.SwerveDrive.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);

    addCommands(
        new EjectCube(intake, IntakeConstants.hightakeSpeed).until(intake::outakeAutoDone),
        new Mobility(swerve),
        new GoIntake(pivot)
            .alongWith(new IntakeCube(intake).until(intake::intakeAutoDone).withTimeout(1)),
        new GoHome(pivot),
        swerveControllerCommand,
        new EjectCube(intake, IntakeConstants.hightakeSpeed).until(intake::outakeAutoDone));
  }
}
