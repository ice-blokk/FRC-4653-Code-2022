// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.util.Limelight;

public class DefaultShoot extends CommandBase {
  
  private Limelight limelight;
  private BooleanSupplier shoot;
  private Shooter shooter;
  private double distanceToTarget, angleToGoal;

  public DefaultShoot(Limelight limelight, BooleanSupplier shoot, Shooter shooter) {
    this.limelight = limelight;
    this.shoot = shoot;
    this.shooter = shooter;
    
    addRequirements(shooter);
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {

    angleToGoal = (Constants.kLimelightAngle + limelight.getY()) * (Math.PI / 180);

    distanceToTarget = (Constants.kTargetAngle - Constants.kLimelightHeight) / Math.tan(angleToGoal);

  }


  @Override
  public void end(boolean interrupted) {}


  @Override
  public boolean isFinished() {
    return false;
  }
}
