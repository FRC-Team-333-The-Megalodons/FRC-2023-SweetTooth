// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake.IntakeConstants.PivotConstants;
import frc.robot.commands.GoIntake;
import frc.robot.Constants.Intake.IntakeConstants;
import frc.robot.Constants.Intake.IntakeConstants.IntakeIDs;
//import frc.robot.Constants.Intake.IntakeConstants.PivotConstants;


/** Add your docs here. */
public  class Pivot extends SubsystemBase {
    private CANSparkMax pivotMotor;
    private AbsoluteEncoder pivotEncoder;
    private PIDController pivotPidController; 

    public Pivot(){
    
         pivotMotor = new CANSparkMax(IntakeIDs.pivotID, MotorType.kBrushless);
         pivotMotor.setIdleMode(IdleMode.kBrake);
         pivotEncoder = pivotMotor.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle);
         pivotPidController = new PIDController(1, 0.1, 0);
         //yeah idk why its red
    }


    public void pivotUp(){ 
        pivotMotor.set(PivotConstants.pivotSpeed);
    }

    public void pivotDown(){
        pivotMotor.set(-PivotConstants.pivotSpeed);
    }
    public void noPivot() {
        pivotMotor.set(0);
    }
    
    public double getPosition() {
    return pivotEncoder.getPosition();
  }
    public void goHome(){
      pivotMotor.set(pivotPidController.calculate(getPosition(), -IntakeConstants.PivotConstants.homeSetpoint));
    }
    public void goIntake(){
      pivotMotor.set(pivotPidController.calculate(getPosition(), IntakeConstants.PivotConstants.intakeSetpoint));
    }
  
        
    /*
    /* //
     * PUT THE ENCODER VALUES AND POSITIONS
     */// hi eman
     @Override
      public void periodic() {
        SmartDashboard.putNumber("Intake Position", getPosition());
        super.periodic();
      }
}

//0.0321
//0.4243
//intakw id 4
//pivot id  2
//haiiiii emannual jefferson can you lend me 150 bucks NO
//haiiiii emannual jefferson can you lend me 150 bucks NO