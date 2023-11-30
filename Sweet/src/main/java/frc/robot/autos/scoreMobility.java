// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.GoScoreHybrid;
import frc.robot.commands.GoHome;
import frc.robot.autos.mobility;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scoreMobility extends SequentialCommandGroup {
  public scoreMobility(Intake m_Intake, Pivot m_Pivot, Swerve m_Mobiloty) {
  }

  /** Creates a new scoreMobility. */
  public class justScore extends SequentialCommandGroup {
    public justScore(Intake m_Intake, Pivot m_Pivot, Swerve m_Mobiloty) {
        addCommands(
          //new GoHome(m_Pivot),
          new GoScoreHybrid(m_Pivot, m_Intake),
          new GoHome(m_Pivot),
          new mobility(m_Mobiloty),
          new GoHome(m_Pivot)
        );
    }
  }
}
