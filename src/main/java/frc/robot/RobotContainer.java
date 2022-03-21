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
import frc.robot.commands.autocommands.DriveThenShootOneBall;
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
import frc.robot.util.Pixy2;
import frc.robot.util.Pixy2Obj;

public class RobotContainer {

  private Drivetrain drivetrain;
  private Transport transport;
  private Intake intake;
  private Shooter shooter;
  private Turret turret;
  private Climbers climber;

  private Limelight limelight;
  private Pixy2Obj pixy;
  
  private Joystick stick;
  private XboxController xbox;

  private SendableChooser<Command> chooser;
  private SendableChooser<Enum> ballChooser;

  public RobotContainer() {

    drivetrain = new Drivetrain();
    transport = new Transport();
    intake = new Intake();
    shooter = new Shooter();
    turret = new Turret();
    climber = new Climbers();

    limelight = new Limelight();
    pixy = new Pixy2Obj();

    stick = new Joystick(0);
    xbox = new XboxController(1);

    chooser = new SendableChooser<Command>();

    setDefaultCommands();
    configureButtonBindings();
    initializeAutoChooser();
    SmartDashboard.putData(chooser);
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
                                               () -> xbox.getRightTriggerAxis(), // shoot low
                                               () -> xbox.getPOV() == 90, // hood up
                                               () -> xbox.getPOV() == 270, // hood down
                                              limelight, shooter, transport));
    turret.setDefaultCommand(new DefaultRotateTurret(() -> filter(xbox.getRightX()), // rotate using the right stick
                                                     () -> xbox.getLeftBumper(), // automatically aim using left bumper   
                                                     limelight, turret));

    climber.setDefaultCommand(new DefaultClimb(() -> xbox.getLeftY(), //climber up and down
                                               () -> xbox.getPOV() == 0, // hooks out
                                               () -> xbox.getPOV() == 180,// hooks in
                                               () -> stick.getRawButton(12), // reset climber encoders
                                               climber));
  }


  private void configureButtonBindings() {
    new JoystickButton(xbox, 9).whenPressed(() -> intake.resetArmEncoder(), intake);
  }

  public void initializeAutoChooser() {
    chooser.setDefaultOption("Nothing", null);

    chooser.addOption("Drive Forward", new DriveOutOfStart(drivetrain));

    //chooser.addOption("Drive then Intake then Shoot", new IntakeBallAndShoot(drivetrain, intake, transport, shooter, turret, limelight));

    chooser.addOption("Drive then Shoot One Ball", new DriveThenShootOneBall(drivetrain, shooter, transport, turret, limelight));

    SmartDashboard.putData(chooser);
  }

  public void initializeBallColorChooser() {
    ballChooser.setDefaultOption("Red Ball lmao", Constants.BallColor.RED);
    
    ballChooser.addOption("Blue Ball lmao", Constants.BallColor.BLUE);

    SmartDashboard.putData(ballChooser);
  }

  public Enum getBallColor() {
    return ballChooser.getSelected();
  }

  public Command getAutonomousCommand() {
    return new DriveThenShootOneBall(drivetrain, shooter, transport, turret, limelight);
  }

  public double filter(double value) {
    if(Math.abs(value) > .15) {
      return value;
    }
    else {
      return 0;
    }
  }
}
