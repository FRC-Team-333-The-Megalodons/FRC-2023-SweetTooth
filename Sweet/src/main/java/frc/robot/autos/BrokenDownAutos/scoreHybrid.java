// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.BrokenDownAutos;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.GoHomePosition;
import frc.robot.commands.GoScoreHybrid;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class scoreHybrid extends SequentialCommandGroup {
  public scoreHybrid(Pivot m_Pivot, Intake m_Intake) {
    addCommands(
      new GoScoreHybrid(m_Pivot, m_Intake),
      new GoHomePosition(m_Pivot)
    );
  }

public scoreHybrid(Swerve s_Swerve, Intake s_Intake, Pivot s_Pivot) {
}
}
