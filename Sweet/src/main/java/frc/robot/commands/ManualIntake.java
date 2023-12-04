// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

/*
 * Command is in charge of manual intake in
 * runs on hold kR1
 */
public class ManualIntake extends CommandBase {
  private final Intake s_Intake;

  public ManualIntake(Intake intake) {
    addRequirements(intake);
    this.s_Intake = intake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_Intake.intake();
  }

  @Override
  public void end(boolean interrupted) {
    s_Intake.noIntake();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
