package frc.robot.autos;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeConstants;
import frc.robot.commands.EjectCube;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class PathPlannerTest extends SequentialCommandGroup {
  public PathPlannerTest(Swerve s_Swerve, Intake intake) {

    PathPlannerTrajectory examplePath =
        PathPlanner.loadPath("Example Path", new PathConstraints(4, 3));

    var thetaController =
        new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController,
            0,
            0,
            Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            examplePath,
            s_Swerve::getPose,
            Constants.SwerveDrive.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            s_Swerve::setModuleStates,
            s_Swerve);

    addCommands(
        new InstantCommand(() -> s_Swerve.resetOdometry(examplePath.getInitialPose())),
        swerveControllerCommand,
        new EjectCube(intake, IntakeConstants.midtakeSpeed).until(intake::outakeAutoDone));
  }
}
