package frc.robot.autos;

import frc.robot.Constants;
import frc.robot.commands.GoIntake;
import frc.robot.commands.IntakeCube;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class MultiPath extends SequentialCommandGroup {
    public MultiPath(Swerve swerve, Pivot pivot, Intake intake){

        PathPlannerTrajectory examplePath = PathPlanner.loadPath("Multi Path", new PathConstraints(4, 3));

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed Marker 1"));
        eventMap.put("Intake Down", new GoIntake(pivot).alongWith(new IntakeCube(intake).until(intake::intakeAutoDone).withTimeout(1)));

        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                examplePath,
                swerve::getPose,
                Constants.SwerveDrive.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                swerve::setModuleStates,
                swerve);
        
        FollowPathWithEvents command = new FollowPathWithEvents(
            (Command) examplePath, 
            examplePath.getMarkers(), 
            eventMap
        );


        addCommands(
        );
    }
}