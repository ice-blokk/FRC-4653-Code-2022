// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoroutines;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autocommands.AutoDrive;
import frc.robot.commands.autocommands.AutoShoot;
import frc.robot.commands.autocommands.AutoTurn;
import frc.robot.commands.defaultcommands.DefaultIntake;
import frc.robot.commands.defaultcommands.DefaultRotateTurret;
import frc.robot.commands.defaultcommands.DefaultShoot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Transport;
import frc.robot.subsystems.Turret;
import frc.robot.util.Limelight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeBallAndShoot extends SequentialCommandGroup {

  public IntakeBallAndShoot(Drivetrain drivetrain, Intake intake, Transport transport, Shooter shooter, Turret turret, Limelight limelight){

    addCommands(

      new InstantCommand(() -> drivetrain.zeroHeading()),

      new RunCommand(() -> intake.armIn(), intake).withInterrupt(() -> intake.isArmOut()).withTimeout(.75),

      new InstantCommand(() -> intake.armOff()),

      new ParallelRaceGroup(
        new DefaultIntake(() -> true, () -> false, () -> false, () -> false, intake),
        new AutoDrive(-.5, 2.25, drivetrain)
      ),

      new AutoDrive(.5, -.75, drivetrain),

      new AutoTurn(140, -.4, drivetrain),

      new InstantCommand(() -> drivetrain.zeroHeading()),
      new InstantCommand(() -> drivetrain.zeroHeading()),

      //new AutoDrive(-.5, 1.25, drivetrain),

      new InstantCommand(() -> intake.intakeOff(), intake),

      new DefaultRotateTurret(() -> 0, () -> true, limelight, turret).withTimeout(.5),

      new WaitCommand(2),

      new AutoShoot(4, shooter, transport, limelight),

      new AutoDrive(.5, -1, drivetrain)

    );
  }

  
}

