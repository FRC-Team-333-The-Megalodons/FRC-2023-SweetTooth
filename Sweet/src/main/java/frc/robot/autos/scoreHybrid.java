// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;
import frc.robot.subsystems.Pivot;
import frc.robot.commands.GoHome;
import frc.robot.commands.GoScoreHybrid;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class scoreHybrid extends SequentialCommandGroup {
  /** Creates a new scoreHybrid. */
  public scoreHybrid(Pivot m_Pivot, Intake m_Intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new GoScoreHybrid(m_Pivot, m_Intake),
      new GoHome(m_Pivot)
    );
  }
}
