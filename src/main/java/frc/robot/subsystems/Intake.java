// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final TalonSRX intake;
  private final CANSparkMax arm;

  public Intake() {
    intake = new TalonSRX(Constants.INTAKE_ADDRESS);
    arm = new CANSparkMax(Constants.ARM_ADDRESS, MotorType.kBrushless);

    intake.setNeutralMode(NeutralMode.Brake);

    arm.setIdleMode(IdleMode.kBrake);

    arm.setSoftLimit(SoftLimitDirection.kForward, (float)Constants.kIntakeArmOutSoftLimit);
    arm.setSoftLimit(SoftLimitDirection.kReverse, (float)Constants.kIntakeArmInSoftLimit);
  }

  // Check if motor is at its forward soft limit
  public boolean isArmOut() {
    return arm.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  // Check if motor is at its reverse soft limit
  public boolean isArmIn() {
    return arm.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
  }

  public void intakeIn() {
    //if(isArmOut()) { // if arm isn't fully out, the chain will break
      intake.set(ControlMode.PercentOutput, .6);
    //}
  }

  public void intakeOut() {
    //if(isArmOut()) { // if arm isn't fully out, the chain will break
      intake.set(ControlMode.PercentOutput, -.6);
    //}
  }

  public void intakeOff() {
    intake.set(ControlMode.PercentOutput, 0);
  }

  public void armOut() {
    if(!isArmOut()) {
      arm.set(-1);
    }
  }

  public void armIn() {
    if(!isArmIn()) {
      arm.set(1);
    }
  }

  public void armOff() {
    arm.set(0);
  }

  @Override
  public void periodic() {

  }
}
