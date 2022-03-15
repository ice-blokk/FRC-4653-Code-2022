// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;
import frc.robot.util.Limelight;

public class DefaultShoot extends CommandBase {
  
  private Limelight limelight;
  private BooleanSupplier shoot, up, down;
  private DoubleSupplier shootLow;
  private Shooter shooter;
  private Transport transport;
  private double distanceToTarget, angleToGoal, hoodIncrement, manualAngle, calculatedAngle;

  public DefaultShoot(BooleanSupplier shoot, DoubleSupplier shootLow, BooleanSupplier up, BooleanSupplier down, Limelight limelight, Shooter shooter, Transport transport) {
    this.shoot = shoot;
    this.limelight = limelight;
    this.shooter = shooter;
    this.transport = transport;
    this.up = up;
    this.down = down;
    this.shootLow = shootLow;

    hoodIncrement = 1;
    manualAngle = shooter.getHoodPosition();
    calculatedAngle = 0;
    
    addRequirements(shooter);
  }


  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {

    angleToGoal = (Constants.kLimelightAngle + limelight.getY()) * (Math.PI / 180); // in radians

    distanceToTarget = (Constants.kTargetHeight - Constants.kLimelightHeight) / Math.tan(angleToGoal); // in inches

    calculatedAngle = (0.01667*(distanceToTarget)*(distanceToTarget)) - (3.167 * (distanceToTarget)) + 200.00; 

    if(shoot.getAsBoolean()) {  
      //shooter.setShooterOpenLoop(.9);
      shooter.setHoodAngle(calculatedAngle);
      shooter.setShooterRPM(3100);
      if(shooter.getShooterRPM() > 3000) {
        transport.setFeeder(-1);
      }
      
    }
    else if(shootLow.getAsDouble() > .5) {
      shooter.setShooterOpenLoop(.3);
      shooter.setHoodAngle(90);
    }
    else {
      shooter.setShooterOpenLoop(0);
    }

    if(up.getAsBoolean()) {
      manualAngle += hoodIncrement;
      //shooter.setHood(manualAngle)
      ;
    }
    else if (down.getAsBoolean()) {
      manualAngle -= hoodIncrement;
      //shooter.setHood(manualAngle);
    }

    

    //shooter.setHood(.5);
    SmartDashboard.putNumber("Hood Angle", shooter.getHoodAngle());
    SmartDashboard.putNumber("Hood Position", shooter.getHoodPosition());
    SmartDashboard.putNumber("Distance to Target", distanceToTarget);
    SmartDashboard.putNumber("Angle to Goal", angleToGoal);
    SmartDashboard.putNumber("Manual Angle", manualAngle);
    SmartDashboard.putNumber("Calculate Anglw", calculatedAngle);
  }


  @Override
  public void end(boolean interrupted) {
    shooter.setShooterOpenLoop(0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
