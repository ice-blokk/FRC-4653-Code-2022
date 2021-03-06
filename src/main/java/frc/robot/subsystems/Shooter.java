// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax leadShooter, followerShooter;
  private Servo hoodMover;

  public Shooter() {
    leadShooter = new CANSparkMax(Constants.LEAD_SHOOTER_ADDRESS, MotorType.kBrushless);
    followerShooter = new CANSparkMax(Constants.FOLLOWER_SHOOTER_ADDRESS, MotorType.kBrushless);

    hoodMover = new Servo(Constants.HOOD_ADJUSTER_ADDRESS);

    leadShooter.setInverted(true);
    followerShooter.follow(leadShooter, true);

    leadShooter.setIdleMode(IdleMode.kCoast);
    followerShooter.setIdleMode(IdleMode.kCoast);

    leadShooter.getPIDController().setP(Constants.kShooterP); //make in constants
    leadShooter.getPIDController().setI(Constants.kShooterI);
    //leadShooter.getPIDController().setD(Constants.kShooterD);
    leadShooter.getPIDController().setFF(Constants.kShooterF);
    leadShooter.getPIDController().setOutputRange(-1, 1);

  }

  public void setHood(int position){
    //hoodMover.setPosition(position);
    hoodMover.setRaw(position);
  }

  public void setHoodAngle(double degrees) {
    //if(degrees <= 110 && degrees >= 0) { // limits of the hood
      hoodMover.setAngle(180 - degrees);
    //}
  }

  public double getHoodAngle() {
    return 180 - hoodMover.getAngle();
  }

  public double getActualHoodAngle() {
    return hoodMover.getAngle();
  }

  public double getHoodPosition() {
    return hoodMover.get();
  }

  public void setShooterRPM(double rpm){
    leadShooter.getPIDController().setReference(rpm, ControlType.kVelocity);
  }

  public void setShooterOpenLoop(double power) {
    leadShooter.set(power);
  }

  public double getShooterRPM() {
    return leadShooter.getEncoder().getVelocity();
  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Lead Shooter RPM", leadShooter.getEncoder().getVelocity());
  }
}
