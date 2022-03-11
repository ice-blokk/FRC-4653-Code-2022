// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autocommands.DriveOutOfStart;
import frc.robot.commands.autocommands.IntakeBallAndShoot;
import frc.robot.commands.defaultcommands.DefaultArcadeDrive;
import frc.robot.commands.defaultcommands.DefaultClimb;
import frc.robot.commands.defaultcommands.DefaultFeedTransport;
import frc.robot.commands.defaultcommands.DefaultIntake;
import frc.robot.commands.defaultcommands.DefaultRotateTurret;
import frc.robot.commands.defaultcommands.DefaultShoot;
import frc.robot.subsystems.Climbers;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.Turret;
import frc.robot.util.Limelight;

public class RobotContainer {

  private Drivetrain drivetrain;
  private Transport transport;
  private Intake intake;
  private Shooter shooter;
  private Turret turret;
  private Climbers climber;

  private Limelight limelight;
  
  private Joystick stick;
  private XboxController xbox;

  private SendableChooser<Command> chooser;

  public RobotContainer() {

    drivetrain = new Drivetrain();
    transport = new Transport();
    intake = new Intake();
    shooter = new Shooter();
    turret = new Turret();
    climber = new Climbers();


    limelight = new Limelight();

    stick = new Joystick(0);
    xbox = new XboxController(1);

    chooser = new SendableChooser<Command>();

    setDefaultCommands();
    configureButtonBindings();
    initializeAutoChooser();
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new DefaultArcadeDrive(() -> stick.getY(), // speed
                                                        () -> stick.getTwist(), // rotation
                                                        () -> stick.getTrigger(), // invert
                                                        drivetrain));
    
    transport.setDefaultCommand(new DefaultFeedTransport(() -> xbox.getStartButton(), // up
                                                         () -> xbox.getBackButton(), // down
                                                         transport));
    
    intake.setDefaultCommand(new DefaultIntake(() -> xbox.getXButton(), // in 
                                               () -> xbox.getBButton(), // out
                                               () -> xbox.getYButton(), // arm up
                                               () -> xbox.getAButton(), // arm down
                                               intake));
    shooter.setDefaultCommand(new DefaultShoot(() -> xbox.getRightBumper(), // shoot
                                               () -> xbox.getPOV() == 0, // hood up
                                               () -> xbox.getPOV() == 180, // hood down
                                              limelight, shooter, transport));
    turret.setDefaultCommand(new DefaultRotateTurret(() -> filter(xbox.getRightX()), // rotate using the right stick
                                                     () -> xbox.getLeftBumper(), // automatically aim using left bumper   
                                                     limelight, turret));

    climber.setDefaultCommand(new DefaultClimb(() -> stick.getRawButton(8), //climber up
                                               () -> stick.getRawButton(7), //climber down
                                               () -> stick.getRawButton(12), // reset climber encoders
                                               climber));
  }

  private void configureButtonBindings() {
    new JoystickButton(xbox, 9).whenPressed(() -> intake.resetArmEncoder(), intake);
  }

  private void initializeAutoChooser() {
    chooser.setDefaultOption("Nothing", null);

    chooser.addOption("Drive Forward", new DriveOutOfStart(drivetrain));

    chooser.addOption("Drive then Intake then Shoot", new IntakeBallAndShoot(drivetrain, intake, transport, shooter, turret, limelight));

    SmartDashboard.putData(chooser);
  }

  public Command getAutonomousCommand() {
    return chooser.getSelected();
  }

  public double filter(double value) {
    if(Math.abs(value) > .05) {
      return 0;
    }
    return value;
  }
}
