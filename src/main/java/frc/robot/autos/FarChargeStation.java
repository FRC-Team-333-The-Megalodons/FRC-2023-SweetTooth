package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class FarChargeStation extends SequentialCommandGroup {
    public FarChargeStation(Swerve s_Swerve){
        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.SwerveDrive.swerveKinematics);

        Trajectory trajectory1 =
            TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(new Translation2d(1, 0), new Translation2d(3.5, 0.127)),
                new Pose2d(5.5, -1.3, new Rotation2d(0)),
                config);

        Trajectory trajectory2 =
        TrajectoryGenerator.generateTrajectory(
            new Pose2d(5.5, -1.3, new Rotation2d(0)),
            List.of(new Translation2d(4.5, -1.3), new Translation2d(3.5, -1.4)),
            new Pose2d(1.2, -1.5, new Rotation2d(0)),
            config);

        var routine = trajectory1.concatenate(trajectory2);

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                routine,
                s_Swerve::getPose,
                Constants.SwerveDrive.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                s_Swerve::setModuleStates,
                s_Swerve);


        addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory1.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> s_Swerve.lockWheels())
        );
    }
}