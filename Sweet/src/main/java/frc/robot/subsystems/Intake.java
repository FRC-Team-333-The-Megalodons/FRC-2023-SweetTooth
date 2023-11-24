// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LEDStrip;
import frc.lib.util.LEDStrip.FancyLED;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.Intake.IntakeConstants;
import frc.robot.Constants.Intake.IntakeConstants.IntakeIDs;

public class Intake extends SubsystemBase {

  private CANSparkMax intakeMotor;
  private LEDStrip led;

  /** Creates a new Intake. */
  public Intake() {
     intakeMotor = new CANSparkMax(IntakeIDs.intakeID, MotorType.kBrushless);
     led = new LEDStrip(0, 400);

     intakeMotor.setIdleMode(IdleMode.kCoast);
  }

  public void intake() { 
    intakeMotor.set(-IntakeConstants.intakeSpeed); 
    led.set(0, 255, 0);
  }
  
  public void outake(double value) { 
    intakeMotor.set(value); 
    led.set(255, 0, 0);
  }

  public void notake() {
    intakeMotor.set(0);
    //led.blink(0, 0, 0); // pink: 255, 105, 180 cooten candy blue: 160, 217, 239
    led.setFancyDualLayer(FancyLED.KNIGHT_RIDER, 255, 105, 180, 160, 217, 239);
  }

  public boolean intakeAutoDone() {
    if (intakeMotor.getEncoder().getPosition() <= -AutoConstants.midIntakeSetpoint) {
      return true;
    }
    return false;
  }

  public boolean outakeAutoDone() {
    if (intakeMotor.getEncoder().getPosition() >= AutoConstants.midIntakeSetpoint) {
      return true;
    }
    return false;
  }

  public void resetIntakeEncoder() {
    intakeMotor.getEncoder().setPosition(0);
  }

  public void blinkPurple() {
    led.blink(106, 13, 173);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Intake Motor Pos", intakeMotor.getEncoder().getPosition());
  }
}