// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
/*
 * This command is bringing the pivot to the home position 
 * On hold kR2
 */
public class GoHomePosition extends CommandBase {
  private final Pivot s_Pivot;

  public GoHomePosition(Pivot pivot) {
    this.s_Pivot = pivot;
    addRequirements(s_Pivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_Pivot.goHome();
  }

  @Override
  public void end(boolean interrupted) {
    s_Pivot.noPivot();
  }

  @Override
  public boolean isFinished() {
    return s_Pivot.isHome();
  }
}
