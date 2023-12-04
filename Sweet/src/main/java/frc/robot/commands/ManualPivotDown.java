// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;

/*
 * This command is in charge of manualy moving the pivot down using kCircle
 * runs on hold kCircle
 */
public class ManualPivotDown extends CommandBase {
  private final Pivot s_Pivot;

  public ManualPivotDown(Pivot pivot) {
  addRequirements(pivot);
  this.s_Pivot = pivot;
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    s_Pivot.pivotDown();
  }

  @Override
  public void end(boolean interrupted) {
    s_Pivot.noPivot();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
