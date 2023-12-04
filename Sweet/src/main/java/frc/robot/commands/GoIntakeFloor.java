// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
/*
 * Command is automatically bringing Pivot to the intake position and intakes the cube
 * runs on hold the touch pad
 */
public class GoIntakeFloor extends CommandBase {
  private final Pivot s_Pivot;
  private final Intake s_Intake;
  public GoIntakeFloor(Pivot pivot, Intake intake) {
    addRequirements(pivot, intake);
    this.s_Intake = intake;
    this.s_Pivot = pivot;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_Pivot.goIntake();
    s_Intake.intake();
  }

  @Override
  public void end(boolean interrupted) {
    s_Intake.noIntake();
    s_Pivot.noPivot();
  }

  @Override
  public boolean isFinished() {
    return s_Pivot.isIntaking();
  }
}
