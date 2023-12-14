// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Constants.Intake.IntakeConstants;
import frc.robot.commands.EjectCube;
import frc.robot.commands.IntakeCube;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
public class MultiPath extends SequentialCommandGroup {
  /** Creates a new MultiPath. */
  public MultiPath(Swerve swerve, Intake intake) {

    PathPlannerTrajectory path1 = PathPlanner.loadPath("Forward", new PathConstraints(3, 2));
    PathPlannerTrajectory path2 = PathPlanner.loadPath("Backward", new PathConstraints(3, 2));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed Marker 1"));

    var thetaController =
    new ProfiledPIDController(
        Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand1 =
        new SwerveControllerCommand(
            path1,
            swerve::getPose,
            Constants.SwerveDrive.swerveKinematics,
            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
            thetaController,
            swerve::setModuleStates,
            swerve);
    SwerveControllerCommand swerveControllerCommand2 =
      new SwerveControllerCommand(
          path2,
          swerve::getPose,
          Constants.SwerveDrive.swerveKinematics,
          new PIDController(Constants.AutoConstants.kPXController, 0, 0),
          new PIDController(Constants.AutoConstants.kPYController, 0, 0),
          thetaController,
          swerve::setModuleStates,
          swerve);

    // FollowPathWithEvents command = new FollowPathWithEvents(
    //   swerveControllerCommand, 
    //   path1.getMarkers(), 
    //   eventMap
    // );
        
    addCommands(
      new EjectCube(intake, IntakeConstants.midtakeSpeed).until(intake::outakeAutoDone),
      new InstantCommand(() -> swerve.resetOdometry(path1.getInitialHolonomicPose())),
      swerveControllerCommand1,
      new IntakeCube(intake).until(intake::intakeAutoDone),
      swerveControllerCommand2,
      new EjectCube(intake, IntakeConstants.midtakeSpeed).until(intake::outakeAutoDone)
      );
  }
}
