// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Turret;
import frc.robot.util.Limelight;

public class DefaultRotateTurret extends CommandBase {

  private Turret turret;
  private Limelight limelight;

  private PIDController pid;

  private DoubleSupplier rotate;
  private BooleanSupplier target;

  private double turretTargetPower;

  public DefaultRotateTurret(DoubleSupplier rotate, BooleanSupplier target, Limelight limelight, Turret turret) {
    this.turret = turret;
    this.rotate = rotate;
    this.target = target;
    this.limelight = limelight;

    pid = new PIDController(.1, 0, 0);
    pid.setSetpoint(0);

    addRequirements(turret);
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    //SmartDashboard.putNumber("stick", rotate.getAsDouble());
    if(Math.abs(rotate.getAsDouble()) > .1) {
      turret.setOpenLoop(rotate.getAsDouble() * -1);
    }
    else if(target.getAsBoolean()) {
      
      turretTargetPower = pid.calculate(limelight.getX());
      
      if(turretTargetPower > 1) {
        turretTargetPower = 1;
      }
      else if(turretTargetPower < -1) {
        turretTargetPower = -1;
      }

      turret.setOpenLoop(turretTargetPower);

    }
    else {
      turret.setOpenLoop(0);
    }

    SmartDashboard.putNumber("Turret Target Power", turretTargetPower);
  
  } // end of execute

  @Override
  public void end(boolean interrupted) {
    turret.setOpenLoop(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
