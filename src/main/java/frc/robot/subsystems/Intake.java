// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Pixy2Obj;

public class Intake extends SubsystemBase {

  private final TalonSRX innerIntake, outerIntake, arm;
  private Pixy2Obj pixy;
  // private final DigitalInput inLimit, outLimit;

  public Intake() {
    innerIntake = new TalonSRX(Constants.INTAKE_INNER_ADDRESS);
    outerIntake = new TalonSRX(Constants.INTAKE_OUTER_ADDRESS);
    arm = new TalonSRX(Constants.ARM_ADDRESS);

    // inLimit = new DigitalInput(Constants.INTAKE_HALL_EFFECT_IN);
    // outLimit = new DigitalInput(Constants.INTAKE_HALL_EFFECT_OUT);


    innerIntake.setNeutralMode(NeutralMode.Brake);
    outerIntake.follow(innerIntake);
    innerIntake.setInverted(true);

    arm.setNeutralMode(NeutralMode.Brake);

    // arm.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.kIntakeArmOutSoftLimit);
    // arm.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.kIntakeArmInSoftLimit);
  }

  // Check if motor is at its forward soft limit
  public boolean isArmOut() {
    //return arm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    //return arm.getEncoder().getPosition() < -225;
    //return !outLimit.get();
    return false;
  }

  // Check if motor is at its reverse soft limit
  public boolean isArmIn() {
    //return arm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
    //return arm.getEncoder().getPosition() > 0;
    //return !inLimit.get();
    return false;
  }

  public void resetArmEncoder() {
    
  }

  public void intakeIn() {
    //if(isArmOut()) { // if arm isn't fully out, the chain will break
      innerIntake.set(ControlMode.PercentOutput, -.9);
      outerIntake.set(ControlMode.PercentOutput, -.9);
    //}
  }

  public void intakeOut() {
    //if(isArmOut()) { // if arm isn't fully out, the chain will break
      innerIntake.set(ControlMode.PercentOutput, .9);
      outerIntake.set(ControlMode.PercentOutput, .9);
    //}
  }

  public void intakeOff() {
    innerIntake.set(ControlMode.PercentOutput, 0);
    outerIntake.set(ControlMode.PercentOutput, 0);
  }

  public void armOut() {
    if(!isArmOut()) {
      arm.set(ControlMode.PercentOutput, .5);
    }
  }

  public void armIn() {
    if(!isArmIn()) {
      arm.set(ControlMode.PercentOutput, -.5);
    }
  }

  public void armOff() {
    arm.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Arm Encoder", arm.getSelectedSensorPosition());
    SmartDashboard.putBoolean("Intake Arm Out Soft Limit", isArmOut());
    SmartDashboard.putBoolean("INtake Arm In Soft Limit", isArmIn());
  }
}
