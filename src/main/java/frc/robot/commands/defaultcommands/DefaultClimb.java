// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climbers;

public class DefaultClimb extends CommandBase {

  private Climbers climber;
  private BooleanSupplier resetEncoders, out, in;
  private DoubleSupplier updown;
  private double position, increment, power;

  /** Creates a new DefaultClimb. */
  public DefaultClimb(DoubleSupplier updown, BooleanSupplier out, BooleanSupplier in, BooleanSupplier resetEncoders, Climbers climber) {
    this.updown = updown;
    this.out = out;
    this.in = in;
    this.climber = climber;
    this.resetEncoders = resetEncoders;
    
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = climber.getFollowerPosition();
    climber.resetEncoders();
    increment = 2;
    power = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //main climbers
    if(updown.getAsDouble() > .5 && !climber.getLeadTopLimit() && !climber.getFollowerTopLimit()){
      position += increment;
    }
    if(updown.getAsDouble() < -.5 && !climber.getLeadLowerLimit() && !climber.getFollowerLowerLimit()){
      position -= increment;
    }

    //reach climber 
    if(out.getAsBoolean() /*&&  !climber.getReachTopLimit() */){
      power = 1;
    }
    else if(in.getAsBoolean() /*&& !climber.getReachLowerLimit() */){
      power = -1;
    }
    else {
      power = 0;
    }

    if(resetEncoders.getAsBoolean()) {
      climber.resetEncoders();
      position = 0;
    }
  
    climber.setLeadPosition(-position);
    climber.setFollowerPosition(position);
    climber.setReachOpenLoop(power);

    SmartDashboard.putNumber("Climber Increment", position);
    SmartDashboard.putNumber("Power for Reach Climber", power);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setReachOpenLoop(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

