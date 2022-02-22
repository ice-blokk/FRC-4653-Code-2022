// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.defaultcommands.DefaultArcadeDrive;
import frc.robot.commands.defaultcommands.DefaultFeedTransport;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Transport;

public class RobotContainer {

  private Drivetrain drivetrain;
  private Transport transport;
  
  private Joystick stick;
  private XboxController xbox;

  public RobotContainer() {

    drivetrain = new Drivetrain();
    transport = new Transport();

    stick = new Joystick(0);
    xbox = new XboxController(1);

    setDefaultCommands();
    configureButtonBindings();
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new DefaultArcadeDrive(() -> stick.getY(), // speed
                                                        () -> stick.getTwist(), // rotation
                                                        () -> stick.getTrigger(), // invert
                                                        drivetrain));
    transport.setDefaultCommand(new DefaultFeedTransport(() -> xbox.getStartButton(), // up
                                                         () -> xbox.getBackButton(), // down
                                                         transport));
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
