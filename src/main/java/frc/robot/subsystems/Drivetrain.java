// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private CANSparkMax frontLeft, frontRight, backLeft, backRight;

  public Drivetrain() {
    frontLeft = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_ADDRESS, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_ADDRESS, MotorType.kBrushless);
    backLeft = new CANSparkMax(Constants.DRIVE_BACK_LEFT_ADDRESS, MotorType.kBrushless);
    backRight = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_ADDRESS, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
  }
}
