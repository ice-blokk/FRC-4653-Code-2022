// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DefaultShoot extends CommandBase {
  /** Creates a new DefaultShoot. */
  
  private LimeLight limelight;
  private BooleanSupplier shoot;
  private Shooter shooter;
  private double distance;

  public DefaultShoot(Limelight limelight, BooleanSupplier shoot, Shooter shooter) {
    this.limelight = limelight;
    this.shoot = shoot;
    this.shooter = shooter;
    
    addRequirements(shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    distance = (Constants.kLimelightTarget - Constants.kLimelightHeight) / (Math.tan(limelight.getY() + Constants.kLimelightAngle));

    

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
