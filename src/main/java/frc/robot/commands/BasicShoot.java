// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class BasicShoot extends CommandBase {
  
  private Shooter shooter;
  private BooleanSupplier shoot;
  
  public BasicShoot(BooleanSupplier shoot, Shooter shooter) {
    this.shooter = shooter;
    this.shoot = shoot;
    addRequirements(shooter);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    if(shoot.getAsBoolean()) {
      shooter.setShooterOpenLoop(.9);
    }
    else {
      shooter.setShooterOpenLoop(0);
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
