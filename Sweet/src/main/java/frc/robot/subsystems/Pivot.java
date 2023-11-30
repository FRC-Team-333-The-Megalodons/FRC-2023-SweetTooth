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
import frc.robot.commands.GoHome;
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
         //wwopwow
         pivotPidController.enableContinuousInput(0, 1);
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
     /*
     * - The method getPosition is responsible for the pivot encoder (which is located on the bottom churro of the intake)
     * it returns the position of the intake. (does it on default)
     * 
     * - The method goHome is setting the encoder position to go into the home position
     * 
     * - The method goIntake is setting the encoder position to go to a position that is accurate to intake a cube from the flood
     */
    public double getPosition() {
    return pivotEncoder.getPosition();
  }
    public void goHome(){
      pivotMotor.set(pivotPidController.calculate(getPosition(), IntakeConstants.PivotConstants.homeSetpoint));
    }
    public void goIntake(){
      pivotMotor.set(pivotPidController.calculate(getPosition(), IntakeConstants.PivotConstants.intakeSetpoint));
    }
    public void scoreHybrid(){
      pivotMotor.set(pivotPidController.calculate(getPosition(), IntakeConstants.PivotConstants.hybridSetpoint));
    }
     /*
       * A boolean method that is being implimented in the SmartDashboard, if the encoder more r equal the home setpoint, it 
       * returns true, otherwise false
       */
    public boolean isHome() {
      if (pivotEncoder.getPosition() >= PivotConstants.homeSetpoint && pivotEncoder.getPosition() >= 0.0406) 
      { return false; } else { return true; }
      }
    public boolean isIntaking() {
      if (pivotEncoder.getPosition() >= PivotConstants.intakeSetpoint && pivotEncoder.getPosition() >= 0.4242)
      { return true; } else { return false; }
      }
    public boolean isScoringHybrid() {
      if (pivotEncoder.getPosition() >= PivotConstants.intakeSetpoint && pivotEncoder.getPosition() >= 0.38) 
      { return true; }  else { return false ; }
    }
        
    /*
    /* //
     * PUT THE ENCODER VALUES AND POSITIONS
     */// hi eman
     @Override
      public void periodic() {
        SmartDashboard.putNumber("Intake Position", getPosition());
        SmartDashboard.putBoolean("IsHome?", isHome());
        SmartDashboard.putBoolean("Isntaking?", isIntaking());
        super.periodic();
      }
}

//0.0321
//0.4243
//intakw id 4
//pivot id  2
//haiiiii emannual jefferson can you lend me 150 bucks NO
//haiiiii emannual jefferson can you lend me 150 bucks NO