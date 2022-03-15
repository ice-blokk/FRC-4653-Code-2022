// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climbers extends SubsystemBase {

  private CANSparkMax leadClimber, followerClimber;

  public Climbers() {
    leadClimber = new CANSparkMax(Constants.CLIMBER_LEADER_ADDRESS, MotorType.kBrushless);
    followerClimber = new CANSparkMax(Constants.CLIMBER_FOLLOWER_ADDRESS, MotorType.kBrushless);

    leadClimber.restoreFactoryDefaults();
    followerClimber.restoreFactoryDefaults();

    leadClimber.setIdleMode(IdleMode.kBrake);
    followerClimber.setIdleMode(IdleMode.kBrake);

    leadClimber.getPIDController().setP(Constants.leadClimberP);
    //leadClimber.getPIDController().setI(Constants.leadClimberI);
    //leadClimber.getPIDController().setD(Constants.leadClimberD);
    //leadClimber.getPIDController().setFF(Constants.leadClimberF);

    followerClimber.getPIDController().setP(Constants.followerClimberP);
    //followerClimber.getPIDController().setI(Constants.followerClimberI);
    //followerClimber.getPIDController().setD(Constants.followerClimberD);
    //followerClimber.getPIDController().setFF(Constants.followerClimberF);   

  }

  public boolean getLeadTopLimit(){
    //return false;
    return (leadClimber.getEncoder().getPosition() < -110);
  }

  public boolean getLeadLowerLimit(){
    return (leadClimber.getEncoder().getPosition() > 0);
    //return false;
  }

  public boolean getFollowerTopLimit(){
    //return false;
    return (followerClimber.getEncoder().getPosition() > 110);
  }

  public boolean getFollowerLowerLimit(){
    //return false;
    return (followerClimber.getEncoder().getPosition() < 0);
  }

  public void setLeadPosition(double position){
    //leadClimber.getEncoder().setPosition(position);
    leadClimber.getPIDController().setReference(position, ControlType.kPosition);
  }

  public void setFollowerPosition(double position){
    //followerClimber.getEncoder().setPosition(position);
    followerClimber.getPIDController().setReference(position, ControlType.kPosition);
  }

  public double getFollowerPosition() {
    return followerClimber.getEncoder().getPosition();
  }

  public void setLeadOpenLoop(double power) {
    leadClimber.set(power);
  }

  public void setFollowerOpenLoop(double power) {
    followerClimber.set(power);
  }

  public void resetEncoders() {
    leadClimber.getEncoder().setPosition(0);
    followerClimber.getEncoder().setPosition(0);
  }



  @Override
  public void periodic() {

      SmartDashboard.putNumber("Lead Climber Encoder", leadClimber.getEncoder().getPosition());
      SmartDashboard.putNumber("Follower Climber Encoder", followerClimber.getEncoder().getPosition());

      SmartDashboard.putBoolean("F top bool", getFollowerTopLimit());
      SmartDashboard.putBoolean("L top bool", getLeadTopLimit());
      SmartDashboard.putBoolean("F bot bool", getFollowerLowerLimit());
      SmartDashboard.putBoolean("L Bot bool", getLeadLowerLimit());


  }
}
