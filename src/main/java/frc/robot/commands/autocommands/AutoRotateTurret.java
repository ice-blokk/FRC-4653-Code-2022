// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.util.Limelight;

public class AutoRotateTurret extends CommandBase {

  private Turret turret;
  private Limelight limelight;
  private double time, turretTargetPower;
  private Timer timer;
  private PIDController pid;

  public AutoRotateTurret(double time, Turret turret, Limelight limelight) {
    this.turret = turret;
    this.limelight = limelight;
    this.time = time;

    pid = new PIDController(.1, 0, 0);
    pid.setSetpoint(0);

    timer = new Timer();

    addRequirements(turret);
  }

  @Override
  public void initialize() {
    timer.stop();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // get turret power from limelight
    turretTargetPower = pid.calculate(limelight.getX());
    
    // set output range [-1, 1]
    if(turretTargetPower > 1) {
      turretTargetPower = 1;
    }
    else if(turretTargetPower < -1) {
      turretTargetPower = -1;
    }

    // set turret power
    turret.setOpenLoop(turretTargetPower);

  } // end of execute

  @Override
  public void end(boolean interrupted) {
    turret.setOpenLoop(0);
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return timer.get() > time;
  }
}
