// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.BrokenDownAutos.ComeBackMobility;
import frc.robot.commands.GoHomePosition;
import frc.robot.commands.GoScoreHybrid;
import frc.robot.commands.GoIntakeFloor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;

public class ComeBackScoreAuto extends SequentialCommandGroup {
  public ComeBackScoreAuto(Swerve s_Swerve, Pivot s_Pivot, Intake s_Intake) {
    addCommands(
      new Mobility(s_Swerve),
      new GoIntakeFloor(s_Pivot, s_Intake),
      new GoHomePosition(s_Pivot),
      new ComeBackMobility(s_Swerve),
      new GoScoreHybrid(s_Pivot, s_Intake)
    );
  }
}
