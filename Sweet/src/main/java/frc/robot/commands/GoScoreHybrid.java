// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;
/*
 * Command is going to the Hybrid SetPoint and outakes the game piece 
 * runs on hold button kSquare
 */
public class GoScoreHybrid extends CommandBase {

  private final Pivot s_Pivot;
  private final Intake s_Intake;
  public GoScoreHybrid(Pivot pivot, frc.robot.subsystems.Intake outake) {
    addRequirements(pivot, outake);
    this.s_Pivot = pivot;
    this.s_Intake = outake;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    s_Pivot.goScoreHybrid();
    s_Intake.outake();
  }

  @Override
  public void end(boolean interrupted) {
    s_Pivot.noPivot();
    s_Intake.noIntake();
  }

  @Override
  public boolean isFinished() {
    return s_Pivot.isScoringHybrid();
    }
  }

