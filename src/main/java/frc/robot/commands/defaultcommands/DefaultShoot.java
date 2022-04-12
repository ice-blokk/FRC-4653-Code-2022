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
  private double distanceToTarget, angleToGoal, hoodIncrement, manualAngle, calculatedAngle, calculatedRPM, 
  myAngle, myVelocity, myRPM, hoodOffset, angleHeight = -69.0, heightOfTarget = 2.64,
  inchesInMeter = 39.3701, metersInInch = 0.0254;

  public DefaultShoot(BooleanSupplier shoot, DoubleSupplier shootLow, BooleanSupplier up, BooleanSupplier down,
      Limelight limelight, Shooter shooter, Transport transport) {
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
    // calculate the angle [deg] and exit velocity [m/s] needed to hit the middle
    // of the upper hub from a specified distance [m] , height [m] and angle
    // [degrees].
  
         
    //returns angle in degrees
    //we will need to add 45 to this value to account for servo motor offset
    // myAngle = Math.toDegrees(Math.atan((Math.tan(Math.toRadians(angleHeight)) * (distanceToTarget * metersInInch) - 2.0 * heightOfTarget) / -(distanceToTarget * metersInInch)));
  
    //returns velocity in m/s
    // myVelocity = Math.sqrt(-(9.8 * (distanceToTarget * metersInInch) * (distanceToTarget * metersInInch) * (1.0 + Math.tan(Math.toRadians(myAngle)) * Math.tan(Math.toRadians(myAngle)))) / (2.0 * heightOfTarget - 2.0 * (distanceToTarget * metersInInch) * Math.tan(Math.toRadians(myAngle))));

    //converts velocity to rpm using wheel diameter (4.875 inches), meters to inches (39.3701), and time
    // myRPM = myVelocity * (inchesInMeter) * (60.0) * (1.0 / (Math.PI * (4.875 / 2.0))) + 1200;

    // https://www.chiefdelphi.com/t/desmos-trajectory-calculator-for-a-shooter-with-an-adjustable-hood/400024

    // inches per minute to revolutions per minute
    // old angle
    // calculatedAngle = (0.01667*(distanceToTarget)*(distanceToTarget)) - (3.167 *(distanceToTarget)) + 200.00 + 34;// - 30.0; // minus 30 because the servo is new
    
    // calculatedAngle = 263.165-23.4383 *
    // Math.log(111.47*(distanceToTarget)-3677.63);
    // calculatedAngle = -((distanceToTarget)*(distanceToTarget) / 160) +
    // (3*(distanceToTarget) / 8) + 75;
    //calculatedAngle = (43 * (Math.pow(distanceToTarget, 3)) / 63000) - (2473 * (Math.pow(distanceToTarget, 2)) / 12600) + ((22787 * distanceToTarget) / 1260) - (1435 / 3);

    calculatedRPM = 0.0554106 * Math.pow(distanceToTarget, 2.12653) + 2961.58 + 125;
    calculatedAngle = 86.6917 - 0.579964 * Math.pow(distanceToTarget, 0.934045);
    //TanMan's funky shooter stuff

    if (shoot.getAsBoolean()) {
      shooter.setHoodAngle(calculatedAngle);
      shooter.setShooterRPM(calculatedRPM);
      if (shooter.getShooterRPM() > calculatedRPM - 45) {
        transport.setFeeder(-1);
      }

    } else if (shootLow.getAsDouble() > .5) {
      shooter.setShooterOpenLoop(.3);
      shooter.setHoodAngle(70);
      transport.setFeeder(-1);
    } else {
      shooter.setShooterOpenLoop(0);
    }

    if (up.getAsBoolean()) {
      //if(manualAngle <= 180) {
        manualAngle += hoodIncrement;
        shooter.setHoodAngle(manualAngle);
      //}
      
    } else if (down.getAsBoolean()) {
      //if(manualAngle >= 0) {
        manualAngle -= hoodIncrement;
        shooter.setHoodAngle(manualAngle);
      //}
      
    }

    //Brady's funky equation stuff

    // if(shoot.getAsBoolean()) {
    // //shooter.setShooterOpenLoop(.9);
    //   shooter.setHoodAngle(myAngle);
    //   shooter.setShooterRPM(myRPM);
    // if(shooter.getShooterRPM() > myRPM - 100) {
    //   transport.setFeeder(-1);
    // }

    // }
    // else if(shootLow.getAsDouble() > .5) {
    //   shooter.setShooterOpenLoop(.3);
    //   shooter.setHoodAngle(70.0);
    //   transport.setFeeder(-1);
    // }
    // else {
    //   shooter.setShooterOpenLoop(0);
    // }

    // if(up.getAsBoolean()) {
    //   myAngle += hoodIncrement;
    // //shooter.setHood(manualAngle)
    // }
    // else if (down.getAsBoolean()) {
    //   myAngle -= hoodIncrement;
    // //shooter.setHood(manualAngle);
    // }

    SmartDashboard.putNumber("Hood Angle", shooter.getHoodAngle());
    SmartDashboard.putNumber("Hood Position", shooter.getHoodPosition());
    SmartDashboard.putNumber("Distance to Target", distanceToTarget);
    SmartDashboard.putNumber("Angle to Goal", angleToGoal);
    SmartDashboard.putNumber("Manual Angle", manualAngle);
    SmartDashboard.putNumber("Calculate Angle", calculatedAngle);
    SmartDashboard.putNumber("Brady's Angle", myAngle);
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
