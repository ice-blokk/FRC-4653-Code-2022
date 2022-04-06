// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Transport;
import frc.robot.util.Pixy2Obj;

public class AutoPixyFindBall extends CommandBase {

  private Drivetrain drivetrain;
  private Intake intake;
  private Pixy2Obj pixy;
  private Transport transport;

  /** Creates a new AutoPixyFindBall. */
  public AutoPixyFindBall(Drivetrain drivetrain, Intake intake, Transport transport, Pixy2Obj pixy) {
    this.drivetrain = drivetrain;
    this.intake = intake;
    this.transport = transport;
    this.pixy = pixy;

    addRequirements(drivetrain, intake, transport);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pixy.updateValues();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    pixy.updateValues();
    intake.intakeIn();

    if(pixy.pixySig(1) && pixy.getPixyArea() > 50){
      if((pixy.getPixyX() < ((315.0/2.0) - 10.0)) && pixy.getPixyX() > ((315.0/2.0) + 10.0)){ //accounts for 20 x range in middle
        drivetrain.arcadeDrive(0.5, (pixy.getPixyX() - (315.0/2.0)) * 0.004); // drives forward in direction of ball
        pixy.updateValues();
      }

    }
    SmartDashboard.putNumber("Pixy X", pixy.getPixyX());
    SmartDashboard.putNumber("Pixy Area", pixy.getPixyArea());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      drivetrain.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return transport.getBeamBreak();
  }
}
