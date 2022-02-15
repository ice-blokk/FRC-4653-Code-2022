// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final TalonSRX intake, arm;

  public Intake() {
    intake = new TalonSRX(Constants.INTAKE_ADDRESS);
    arm = new TalonSRX(Constants.ARM_ADDRESS);
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

  @Override
  public void periodic() {

  }
}
