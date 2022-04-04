// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoRotateTurret;
import frc.robot.commands.autocommands.AutoShoot;
import frc.robot.commands.defaultcommands.DefaultFeedTransport;
import frc.robot.commands.defaultcommands.DefaultRotateTurret;
import frc.robot.commands.defaultcommands.DefaultShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.Turret;
import frc.robot.util.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveThenShootOneBall extends SequentialCommandGroup {
  /** Creates a new DriveThenShootOneBall. */
  public DriveThenShootOneBall(Drivetrain drivetrain, Shooter shooter, Transport transport, Turret turret, Limelight limelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoDrive(.5, -1.0, drivetrain),

      new AutoRotateTurret(2, turret, limelight),

      new WaitCommand(.3),

      new AutoShoot(3, shooter, transport, limelight),

      new AutoDrive(.5, -1, drivetrain)

      //new AutoDrive(.5, -1, drivetrain)
    );
  }
}
