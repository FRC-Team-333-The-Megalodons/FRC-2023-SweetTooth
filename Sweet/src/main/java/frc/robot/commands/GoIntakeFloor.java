// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;

public class GoIntakeFloor extends CommandBase {
  /** Creates a new GoIntakeFloor. */
  private final Pivot s_Pivot;
  private final Intake s_Intake;
  public GoIntakeFloor(Pivot pivot, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot, intake);
    this.s_Intake = intake;
    this.s_Pivot = pivot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Pivot.goIntake();
    s_Intake.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Intake.noIntake();
    s_Pivot.noPivot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Pivot.isIntaking();
  }
}
