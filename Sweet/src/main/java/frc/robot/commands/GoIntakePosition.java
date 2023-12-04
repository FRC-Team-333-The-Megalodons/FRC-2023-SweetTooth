// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
public class GoIntakePosition extends CommandBase {
  private final Pivot s_Pivot;

  /*
   * This class is bringing pivot to the intake position 
   * runs on hold kL2
   */
  public GoIntakePosition(Pivot pivot) {
    this.s_Pivot = pivot;
    addRequirements(s_Pivot);
  }
  
    @Override
    public void initialize() {}
  
    @Override
    public void execute() {
      s_Pivot.goIntake();
    }

    @Override
    public void end(boolean interrupted) {
      s_Pivot.noPivot();
    }
  
    @Override
    public boolean isFinished() {
      return s_Pivot.isIntaking();
    }
}
