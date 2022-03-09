// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;

public class DefaultClimb extends CommandBase {

  private Climbers climber;
  private BooleanSupplier up, down;
  private double position, increment;

  /** Creates a new DefaultClimb. */
  public DefaultClimb(BooleanSupplier up, BooleanSupplier down, Climbers climber) {
    this.up = up;
    this.down = down;
    this.climber = climber;
    
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = 0;
    increment = 0.0001;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(up.getAsBoolean() && !climber.getLeadTopLimit() && !climber.getFollowerTopLimit()){
      position += increment;
    }
    if(down.getAsBoolean() && !climber.getLeadLowerLimit() && !climber.getFollowerLowerLimit()){
      position -= increment;
    }
  
    climber.setLeadPosition(position);
    climber.setFollowerPosition(position);

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
