// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteLimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret extends SubsystemBase {

  private TalonSRX turret;
  private DigitalInput fwdSwitch, revSwitch; // limit switches
  //  (idk if i'm gonna use this with soft limits or just the hall effects or at all)

  public Turret() {
    turret = new TalonSRX(Constants.TURRET_ADDRESS);

    fwdSwitch = new DigitalInput(Constants.TURRET_FWD_LIMIT_SWITCH);
    revSwitch = new DigitalInput(Constants.TURRET_REV_LIMIT_SWITCH);

    turret.configFactoryDefault();

    turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    turret.setSensorPhase(true);

    // set to brake mode
    turret.setNeutralMode(NeutralMode.Brake);
    
    // config soft limits with encoders
    //turret.configForwardSoftLimitThreshold(Constants.kSoftMaxTurretAngle / (360.0 * Constants.kTurretRotationsPerTick));
    //turret.configReverseSoftLimitThreshold(Constants.kSoftMinTurretAngle / (360.0 * Constants.kTurretRotationsPerTick));

    // enable soft limits
    //turret.configForwardSoftLimitEnable(true);
    //turret.configReverseSoftLimitEnable(true);

    // config PID and feedforward
    turret.config_kP(Constants.kTurretPIDLoopIdx, Constants.kTurretP);
    turret.config_kI(Constants.kTurretPIDLoopIdx, Constants.kTurretI);
    turret.config_kD(Constants.kTurretPIDLoopIdx, Constants.kTurretD);
    turret.config_kF(Constants.kTurretPIDLoopIdx, Constants.kTurretF);
  }

  // Sets turret to desired angle (in position mode)
  public void setDesiredAngle(Rotation2d angle) {
    turret.set(ControlMode.Position, angle.getRadians() / (2 * Math.PI * Constants.kTurretRotationsPerTick));
  }

  public boolean getFwdSwitch() {
    return fwdSwitch.get();
  }

  public boolean getRevSwitch() {
    return revSwitch.get();
  }

  // Manually move the turret (in percent mode)
  public void setOpenLoop(double speed) {
    if(speed > 0) {
      if(!getFwdSwitch()) {
        turret.set(ControlMode.PercentOutput, speed);
      }
      else {
        turret.set(ControlMode.PercentOutput, 0);
      }
    }
    else if(speed < 0){
      if(!getRevSwitch()) {
        turret.set(ControlMode.PercentOutput, speed);
      }
      else {
        turret.set(ControlMode.PercentOutput, 0);
      }
    }
    else {
      turret.set(ControlMode.PercentOutput, 0);
    }
  }

  // Tell the talon it is at a certain angle
  public void resetAngle(Rotation2d actualRotation) {
    turret.set(ControlMode.Position, actualRotation.getRadians() / (2 * Math.PI * Constants.kTurretRotationsPerTick));
  }

  // Returns current angle as Rotation2d
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(Math.toDegrees(Constants.kTurretRotationsPerTick * turret.getSelectedSensorPosition() * 2 * Math.PI));
  }

  public boolean getForwardSoftLimitSwitch() {
    return turret.isFwdLimitSwitchClosed() == 1;
  }

  public boolean getReverseSoftLimitSwitch() {
    return turret.isRevLimitSwitchClosed() == 1;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Angle (Degrees)", getAngle().getDegrees());
    SmartDashboard.putBoolean("Turret Fwd Soft Limit", getForwardSoftLimitSwitch());
    SmartDashboard.putBoolean("Turret Rev Soft Limit", getReverseSoftLimitSwitch());
    SmartDashboard.putBoolean("Turret Fwd Limit Switch", getFwdSwitch());
    SmartDashboard.putBoolean("Turret Rev Limit Switch", getRevSwitch());
  }
}
