// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoTurn;
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
public class ShootThenDriveForward extends SequentialCommandGroup {
  /** Creates a new ShootThenDriveForward. */
  public ShootThenDriveForward(Drivetrain drivetrain, Limelight limelight, Shooter shooter, Transport transport, Turret turret) {
    addCommands(
      
      new DefaultRotateTurret(() -> 0, () -> true, limelight, turret).withTimeout(2),

      new DefaultShoot(() -> true, () -> 0, () -> false, () -> false, limelight, shooter, transport).withTimeout(3),

      new AutoTurn(-90, -.5, drivetrain),

      new AutoDrive(1.5, -.5, drivetrain)
    );
  }
}
