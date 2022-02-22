// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final TalonSRX intake, arm;

  public Intake() {
    intake = new TalonSRX(Constants.INTAKE_ADDRESS);
    arm = new TalonSRX(Constants.ARM_ADDRESS);

    arm.setNeutralMode(NeutralMode.Brake);
    arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void intakeIn() {
    intake.set(ControlMode.PercentOutput, -.5);
  }

  public void intakeOut() {
    intake.set(ControlMode.PercentOutput, .5);
  }

  public void intakeOff() {
    intake.set(ControlMode.PercentOutput, 0);
  }

  public void setArmUp() {
    arm.set(ControlMode.Position, Constants.kArmUpPosition);
  }

  public void setArmDown() {
    arm.set(ControlMode.Position, Constants.kArmDownPosition);
  }

  public void setArmOff() {
    arm.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {

  }
}
