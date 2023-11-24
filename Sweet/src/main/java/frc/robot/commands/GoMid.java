// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;

public class GoMid extends CommandBase {

  private final Pivot m_pivot;

  /** Creates a new PivotArm. */
  public GoMid(Pivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_pivot = pivot;
    addRequirements(m_pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_pivot.pivotToMid();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_pivot.noPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_pivot.atMidSetpoint();
  }
}
