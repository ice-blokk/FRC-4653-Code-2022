// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.util.Pixy2;
import frc.robot.util.Pixy2CCC;
import frc.robot.util.Pixy2Obj;

public class PixyAuto extends CommandBase {
  /** Creates a new PixyAuto. */
  private Drivetrain drivetrain;
  private Intake intake;
  private Pixy2Obj pixy;
  private Enum ballColor;

  private PIDController pid;

  double maxVal = 0, turn, kP;

  public PixyAuto(Enum ballColor, Pixy2Obj pixy, Drivetrain drivetrain, Intake intake) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.pixy = pixy;
    this.ballColor = ballColor;

    pid = new PIDController(.0025, 0, 0);

    addRequirements(drivetrain, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pixy.updateValues();
    kP = 160;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pixy.updateValues();

    if(Math.abs(-(pid.calculate(kP))) > Math.abs(maxVal)) {
      maxVal = -(pid.calculate(kP));
    }


    if(pixy.getPixyX() != 0) {
      kP = (pixy.getPixyX() - 160);
    }
    turn = -(pid.calculate(kP));
    /* turn 
    better */
    
    if(turn > .175) {
      turn += .2;
    }
    else if (turn < -.175) {
      turn += -.2;
    }

    // limit output range
    if (turn > 1) {
      turn = 1;
    }
    else if (turn < -1) {
      turn = -1;
    }
    

    if(ballColor == Constants.BallColor.BLUE) {
      if(pixy.pixySig(1)) {
        drivetrain.arcadeDrive(.4, turn);
      }
      else {
        drivetrain.arcadeDrive(0, 0);
      }
    }
    else if(ballColor == Constants.BallColor.RED) {
      if(pixy.pixySig(2)) {
        drivetrain.arcadeDrive(.4, turn);
      }
      else {
        drivetrain.arcadeDrive(0, 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
