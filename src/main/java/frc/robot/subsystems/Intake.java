// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final TalonSRX intake, arm;
  private final DigitalInput outHallEffect, inHallEffect;

  public Intake() {
    intake = new TalonSRX(Constants.INTAKE_ADDRESS);
    arm = new TalonSRX(Constants.ARM_ADDRESS);

    outHallEffect = new DigitalInput(Constants.INTAKE_OUT_HALLEFFECT_ADDRESS);
    inHallEffect = new DigitalInput(Constants.INTAKE_IN_HALLEFFECT_ADDRESS);

    arm.setNeutralMode(NeutralMode.Brake);
    arm.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // config soft limits with encoders
    arm.configForwardSoftLimitThreshold(Constants.kSoftMaxTurretAngle / (360.0 * Constants.kTurretRotationsPerTick));
    arm.configReverseSoftLimitThreshold(Constants.kSoftMinTurretAngle / (360.0 * Constants.kTurretRotationsPerTick));

    // enable soft limits
    arm.configForwardSoftLimitEnable(true);
    arm.configReverseSoftLimitEnable(true);
  }

  public boolean isArmOut() {
    return outHallEffect.get();
  }

  public boolean isArmIn() {
    return inHallEffect.get();
  }

  public void intakeIn() {
    if(isArmOut()) { // if arm isn't fully out, the chain will break
      intake.set(ControlMode.PercentOutput, -.7);
    }
  }

  public void intakeOut() {
    if(isArmOut()) { // if arm isn't fully out, the chain will break
      intake.set(ControlMode.PercentOutput, .7);
    }
  }

  public void intakeOff() {
    intake.set(ControlMode.PercentOutput, 0);
  }

  public void armOut() {
    if(!isArmOut()) {
      arm.set(ControlMode.PercentOutput, -.5);
    }
  }

  public void armIn() {
    if(!isArmIn()) {
      arm.set(ControlMode.PercentOutput, .5);
    }
  }

  public void armOff() {
    arm.set(ControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {

  }
}
