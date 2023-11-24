// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeCube extends CommandBase {

  private final Intake m_intake;

  /** Creates a new IntakeCube. */
  public IntakeCube(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_intake = intake;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.resetIntakeEncoder();
  }

  // Called every time the schedumler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.intake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.notake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}