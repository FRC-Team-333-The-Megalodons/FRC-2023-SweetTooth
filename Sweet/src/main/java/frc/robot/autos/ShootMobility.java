package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.BrokenDownAutos.MibilityBack;
import frc.robot.commands.ManualOutake;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;

public class ShootMobility extends SequentialCommandGroup {
  public ShootMobility (Swerve s_Swerve, Intake s_Intake) {
    addCommands(
    new ManualOutake(s_Intake),
    new Mobility(s_Swerve),
    new ManualOutake(s_Intake),
    new MibilityBack(s_Swerve)
    );
  }
}
