// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
public class GoIntake extends CommandBase {
  private final Pivot s_Pivot;

  public GoIntake(Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Pivot = pivot;
    addRequirements(s_Pivot);
  }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      s_Pivot.goIntake();
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      s_Pivot.noPivot();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return s_Pivot.isIntaking();
    }
}
