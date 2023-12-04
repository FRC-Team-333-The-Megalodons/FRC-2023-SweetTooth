// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.GoScoreHybrid;
import frc.robot.commands.GoHomePosition;
import frc.robot.commands.GoIntakeFloor;

public class ScoreMobility extends SequentialCommandGroup {
  public ScoreMobility(Intake m_Intake, Pivot m_Pivot, Swerve m_Mobiloty) {
  }


  public class justScore extends SequentialCommandGroup {
    public justScore(Intake m_Intake, Pivot m_Pivot, Swerve m_Mobiloty) {
        addCommands(
          new GoScoreHybrid(m_Pivot, m_Intake),
          new GoHomePosition(m_Pivot),
          new Mobility(m_Mobiloty),
          new GoIntakeFloor(m_Pivot, m_Intake)
        );
    }
  }
}
