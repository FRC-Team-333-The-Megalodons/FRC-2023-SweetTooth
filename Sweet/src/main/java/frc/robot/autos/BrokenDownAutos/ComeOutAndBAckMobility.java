// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos.BrokenDownAutos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autos.Mobility;
import frc.robot.subsystems.Swerve;

public class ComeOutAndBAckMobility extends SequentialCommandGroup {
  public ComeOutAndBAckMobility(Swerve s_Swerve) {
    addCommands(
      new Mobility(s_Swerve),
      new ComeBackMobility(s_Swerve)
    );
  }
}
