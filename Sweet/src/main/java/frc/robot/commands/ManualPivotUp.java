// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Pivot;
import edu.wpi.first.wpilibj2.command.CommandBase;

/* 
 * this command is in charge of pivot going manualy Up
 * runs on while hold kTriangle
 */
public class ManualPivotUp extends CommandBase {
  private final Pivot s_Pivot;

  public ManualPivotUp(Pivot pivot) {
    addRequirements(pivot);
    this.s_Pivot = pivot;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_Pivot.pivotUp();
  }

  @Override
  public void end(boolean interrupted) {
    s_Pivot.noPivot();
  }

  public boolean isFinished() {
    return false;
  }
}
