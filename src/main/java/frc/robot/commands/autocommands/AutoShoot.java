// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;
import frc.robot.util.Limelight;

public class AutoShoot extends CommandBase {
  private Limelight limelight;
  private Shooter shooter;
  private Transport transport;
  private double time, angleToGoal, distanceToTarget, calculatedAngle, calculatedRPM;
  private Timer timer;
  public AutoShoot(double time, Shooter shooter, Transport transport, Limelight limelight) {
    this.shooter = shooter;
    this.transport = transport;
    this.limelight = limelight;
    this.time = time;

    timer = new Timer();
    
    addRequirements(shooter, transport);
  }

  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    angleToGoal = (Constants.kLimelightAngle + limelight.getY()) * (Math.PI / 180); // in radians

    distanceToTarget = (Constants.kTargetHeight - Constants.kLimelightHeight) / Math.tan(angleToGoal); // in inches

    calculatedRPM = 0.0554106 * Math.pow(distanceToTarget, 2.12653) + 2961.58 + 125;
    calculatedAngle = 86.6917 - 0.579964 * Math.pow(distanceToTarget, 0.934045);

    shooter.setHoodAngle(calculatedAngle);
    shooter.setShooterRPM(calculatedRPM);
    if (shooter.getShooterRPM() > calculatedRPM - 45) {
      transport.setFeeder(-1);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooter.setShooterOpenLoop(0);
    transport.setFeeder(0);
  }

  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
