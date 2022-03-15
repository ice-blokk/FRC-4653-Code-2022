// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
      new RunCommand(() -> intake.armOut(), intake).withInterrupt(() -> intake.isArmOut()),

      new ParallelRaceGroup(
        new RunCommand(() -> intake.intakeIn(), intake),
        new AutoDrive(-.5, 1.5, drivetrain)
      ),

      new AutoTurn(120, .5, drivetrain),

      new DefaultRotateTurret(() -> 0, () -> true, limelight, turret).withTimeout(.5),

      new DefaultShoot(() -> true, () -> 0, () -> false, () -> false, limelight, shooter, transport).withTimeout(3)
      
      
      

      


    );
  }

  
}

