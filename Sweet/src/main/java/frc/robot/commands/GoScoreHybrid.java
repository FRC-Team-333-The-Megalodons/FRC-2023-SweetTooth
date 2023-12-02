// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Intake;
import frc.robot.Constants.Intake.IntakeConstants.PivotConstants;
import frc.robot.Constants.Intake.IntakeConstants;
import frc.robot.Constants.Intake.IntakeConstants.IntakeIDs;
public class GoScoreHybrid extends CommandBase {

  private final Pivot s_Pivot;
  private final Intake s_Intake;
  public GoScoreHybrid(Pivot pivot, frc.robot.subsystems.Intake outake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot, outake);
    this.s_Pivot = pivot;
    this.s_Intake = outake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Pivot.goScoreHybrid();
    s_Intake.outake();
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Pivot.noPivot();
    s_Intake.noIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Pivot.isScoringHybrid();
    }
  }

