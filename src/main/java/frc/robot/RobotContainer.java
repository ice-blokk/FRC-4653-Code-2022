// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.defaultcommands.DefaultArcadeDrive;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  private Drivetrain drivetrain;
  private Joystick stick;

  public RobotContainer() {

    drivetrain = new Drivetrain();

    stick = new Joystick(0);

    setDefaultCommands();
    configureButtonBindings();
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new DefaultArcadeDrive(() -> stick.getY(), 
                                                        () -> stick.getTwist(),
                                                        () -> stick.getTrigger(),
                                                        drivetrain));
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
  }
}
