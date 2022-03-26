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
  private double distanceToTarget, angleToGoal, hoodIncrement, manualAngle, calculatedAngle, angleForCalc, speedForCalc, finRPM;

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

    // finding the velocity to use in calculations
    //calculate the angle [deg] and exit velocity [m/s] needed to hit the middle 
    //of the upper hub from a specified distance [m] , height [m] and angle [degrees].

    //distanceToTarget = distanceToTarget * 0.0254;

    //angleForCalc = Math.atan((Math.tan(Math.toRadians(-69)) * (distanceToTarget * 0.0254) - 2 * 2.64) / -(distanceToTarget * 0.0254));

    //speedForCalc = Math.sqrt((-(9.8 * (distanceToTarget * 0.0254) * (distanceToTarget * 0.0254) * (1 + (Math.tan(angleForCalc) * Math.tan(angleForCalc))))) / ((2 * 2.64) - (2 * (distanceToTarget * 0.0254) * Math.tan(angleForCalc))));

    //finRPM = 2.2 * speedForCalc * (39.3701) * (60) * (1 / (Math.PI * 4.875));
    
    //https://www.chiefdelphi.com/t/desmos-trajectory-calculator-for-a-shooter-with-an-adjustable-hood/400024
    
// inches per minute to revolutions per minute
    // old angle
    // calculatedAngle = (0.01667*(distanceToTarget)*(distanceToTarget)) - (3.167 * (distanceToTarget)) + 200.00; 

    calculatedAngle = 263.165-23.4383 * Math.log(111.47*(distanceToTarget)-3677.63);

    if(shoot.getAsBoolean()) {  
      //shooter.setShooterOpenLoop(.9);
      shooter.setHoodAngle(calculatedAngle);
      shooter.setShooterRPM(3200);
      if(shooter.getShooterRPM() > 3100) {
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
      shooter.setHoodAngle(manualAngle)
      ;
    }
    else if (down.getAsBoolean()) {
      manualAngle -= hoodIncrement;
      shooter.setHoodAngle(manualAngle);
    }


    // if(shoot.getAsBoolean()) {  
    //   //shooter.setShooterOpenLoop(.9);
    //   shooter.setHoodAngle(Math.toDegrees(angleForCalc) + 35);
    //   shooter.setShooterRPM(finRPM);
    //   if(shooter.getShooterRPM() > finRPM - 200) {
    //     transport.setFeeder(-1);
    //   }
      
    // }
    // else if(shootLow.getAsDouble() > .5) {
    //   shooter.setShooterOpenLoop(.3);
    //   shooter.setHoodAngle(angleForCalc);
    // }
    // else {
    //   shooter.setShooterOpenLoop(0);
    // }

    // if(up.getAsBoolean()) {
    //   angleForCalc += hoodIncrement;
    //   //shooter.setHood(manualAngle)
    // }
    // else if (down.getAsBoolean()) {
    //   angleForCalc -= hoodIncrement;
    //   //shooter.setHood(manualAngle);
    // }

    

    //shooter.setHood(.5);
    SmartDashboard.putNumber("Hood Angle", shooter.getHoodAngle());
    SmartDashboard.putNumber("Hood Position", shooter.getHoodPosition());
    SmartDashboard.putNumber("Distance to Target", distanceToTarget);
    SmartDashboard.putNumber("Angle to Goal", angleToGoal);
    SmartDashboard.putNumber("Manual Angle", manualAngle);
    SmartDashboard.putNumber("Calculate Anglw", calculatedAngle);
    SmartDashboard.putNumber("angleForCalc", Math.toDegrees(angleForCalc));
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
