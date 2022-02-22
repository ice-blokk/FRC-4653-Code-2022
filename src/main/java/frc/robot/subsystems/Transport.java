// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Transport extends SubsystemBase {

  private TalonSRX transport;
  private DigitalInput beamBreak;

  private double startEncoderValue;

  public Transport() {
    transport = new TalonSRX(Constants.TRANSPORT_ADDRESS);

    beamBreak = new DigitalInput(Constants.TRANSPORT_BEAM_BREAK_ADDRESS);

    transport.setNeutralMode(NeutralMode.Brake);
    transport.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  }

  public void setFeeder(double power) {
    transport.set(ControlMode.PercentOutput, power);
  }

  public double getEncoderValue() {
    return transport.getSelectedSensorPosition();
  }

  public boolean getBeamBreak() {
    return beamBreak.get();
  }

  public double getEncoderDelta() {
    return Math.abs(startEncoderValue - transport.getSelectedSensorPosition());
  }

  public void resetEncoderDelta() {
    startEncoderValue = transport.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Transport Encoder", transport.getSelectedSensorPosition());
    SmartDashboard.putNumber("Transport Encoder Delta", getEncoderDelta());
  }
}
