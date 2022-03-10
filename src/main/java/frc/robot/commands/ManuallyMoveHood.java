// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class ManuallyMoveHood extends CommandBase {
  private Shooter shooter;
  private BooleanSupplier up, down;
  private double position, increment;

  public ManuallyMoveHood(BooleanSupplier up, BooleanSupplier down, Shooter shooter) {
    this.shooter = shooter;
    this.up = up;
    this.down = down;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = 100;
    increment = .1;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(up.getAsBoolean()) {
      position += increment;
    }
    if(down.getAsBoolean()) {
      position -= increment;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
