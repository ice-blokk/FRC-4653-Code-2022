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
  private BooleanSupplier up, down, highUp, highDown;
  private double position, increment, power;

  /** Creates a new DefaultClimb. */
  public DefaultClimb(BooleanSupplier up, BooleanSupplier down, BooleanSupplier highUp, BooleanSupplier highDown, BooleanSupplier out, BooleanSupplier in, BooleanSupplier resetEncoders, Climbers climber) {
    this.up = up;
    this.down = down;
    this.out = out;
    this.in = in;
    this.climber = climber;
    this.resetEncoders = resetEncoders;

    this.highUp = highUp;
    this.highDown = highDown;
    
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    position = climber.getFollowerPosition();
    ///climber.resetEncoders();
    increment = 2;
    power = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //main climbers
    if(up.getAsBoolean() /* && !climber.getLeadTopLimit() && !climber.getFollowerTopLimit() */){
      position += increment;
      // climber.setLeadPosition(-position);
      // climber.setFollowerPosition(position);
      climber.setLeadOpenLoop(-.8);
      climber.setFollowerOpenLoop(-.8);
    }
    else if(down.getAsBoolean()  /*&& !climber.getLeadLowerLimit() && !climber.getFollowerLowerLimit() */){
      position -= increment;
      // climber.setLeadPosition(-position);
      // climber.setFollowerPosition(position);
      climber.setLeadOpenLoop(.8);
      climber.setFollowerOpenLoop(.8);
    }
    else {
      climber.setFollowerOpenLoop(0);
      climber.setLeadOpenLoop(0);
    }

    if(highUp.getAsBoolean()) {
      climber.setHighClimberOpenLoop(-.9);;
    }
    else if (highDown.getAsBoolean()) {
      climber.setHighClimberOpenLoop(.9);
    }
    else {
      climber.setHighClimberOpenLoop(0);
    }

    //reach climber 
    if(out.getAsBoolean() /*&&  !climber.getReachTopLimit() */){
      climber.setReachOpenLoop(1);
    }
    else if(in.getAsBoolean() /*&& !climber.getReachLowerLimit() */){
      climber.setReachOpenLoop(-1);
    }
    else {
      climber.setReachOpenLoop(0);
    }

    if(resetEncoders.getAsBoolean()) {
      position = 0;
      climber.resetEncoders();
    }

    SmartDashboard.putNumber("Climber Increment", position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //climber.setReachOpenLoop(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

