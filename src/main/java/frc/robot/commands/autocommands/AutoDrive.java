// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;



public class AutoDrive extends CommandBase {
  /** Creates a new AutoDrive. */

  private Drivetrain drivetrain;
  private double power, distance, startValue;

  public AutoDrive(double power, double distance, Drivetrain drivetrain) {
    this.power = power;
    this.distance = distance;
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startValue = (drivetrain.getLeftEncoderDistance() + drivetrain.getRightEncoderDistance()) / 2;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.arcadeDrive(power, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(power < 0 && ((drivetrain.getLeftEncoderDistance() + drivetrain.getRightEncoderDistance()) / 2) >= startValue + distance){
      return true;
    }

    if(power > 0 && ((drivetrain.getLeftEncoderDistance() + drivetrain.getRightEncoderDistance()) / 2) <= startValue + distance){
      return true;
    }

    return false;
  }
}
