// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class DefaultIntake extends CommandBase {
  
  private Intake intake;

      BooleanSupplier in, out, armUp, armDown;

  public DefaultIntake(BooleanSupplier in, BooleanSupplier out, BooleanSupplier armUp, BooleanSupplier armDown, Intake intake) {
    this.intake = intake;
    addRequirements(intake);

    this.in = in;
    this.out = out;
    this.armUp = armUp;
    this.armDown = armDown;
  }

  
  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    if(in.getAsBoolean()) {
      //if(intake.isArmOut()) {
      intake.intakeIn();
      //}
    }
    else if (out.getAsBoolean()) {
      //if(intake.isArmOut()) {
        intake.intakeOut();
      //}
    }
    else {
      intake.intakeOff();
    }

    if(armUp.getAsBoolean()) {
      if(!intake.isArmOut()) {
        intake.armOut();
      } 
      else {
        intake.armOff();
      }
    }
    else if(armDown.getAsBoolean()) {
      if(!intake.isArmIn()) {
        intake.armIn();
      }
      else {
        intake.armOff();
      }
    }
    else {
      intake.armOff();
    }
  }

  
  @Override
  public void end(boolean interrupted) {}

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
