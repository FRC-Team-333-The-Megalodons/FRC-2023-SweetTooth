// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.lib.util.LEDStrip;
import frc.robot.Constants.Intake.IntakeConstants;
import frc.robot.Constants.Intake.IntakeConstants.IntakeIDs;

public class Intake extends SubsystemBase{
    private CANSparkMax intakeMotor;
   // private LEDStrip led;
    public Intake() { intakeMotor = new CANSparkMax(IntakeIDs.intakeID, MotorType.kBrushless);
       // led = new LEDStrip(1, 100);
    }
    public void intake() { 
        intakeMotor.set(-IntakeConstants.intakeSpeed); 
       // led.set(255,192,200);
      }
      public void outake() { 
        intakeMotor.set(IntakeConstants.outtakeSpeed); 
       // led.set(100, 100, 100);
      }
      public void noIntake() {
        intakeMotor.set(0);
      }
      @Override //wtf
      public void periodic() {
      }
    public void setDefaultCommand() {
    }
}
