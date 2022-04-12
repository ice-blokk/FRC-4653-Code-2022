// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DefaultArcadeDrive extends CommandBase {

  private Drivetrain drivetrain;
  private DoubleSupplier speed, rotate;
  private BooleanSupplier invert;
  private double sign;

  public DefaultArcadeDrive(DoubleSupplier speed, DoubleSupplier rotate, BooleanSupplier invert, Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.speed = speed;
    this.rotate = rotate;
    this.invert = invert;
    sign = 1;
  }


  @Override
  public void initialize() {}


  @Override
  public void execute() {
    if(rotate.getAsDouble() > 0) {
      sign = 1;
    }
    else {
      sign = -1;
    }
    drivetrain.arcadeDrive(speed.getAsDouble(), rotate.getAsDouble() * rotate.getAsDouble() * sign * .55, invert.getAsBoolean());

    SmartDashboard.putNumber("arcade drive turn", rotate.getAsDouble() * rotate.getAsDouble() * sign * .55);
  }


  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }


  @Override
  public boolean isFinished() {
    return false;
  }
}
