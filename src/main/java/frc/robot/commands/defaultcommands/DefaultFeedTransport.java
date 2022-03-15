// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.defaultcommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Transport;

public class DefaultFeedTransport extends CommandBase {
  
  private Transport transport;

  private BooleanSupplier up, down;
  private Boolean sensorWasTriggered = false;

  public DefaultFeedTransport(BooleanSupplier up, BooleanSupplier down, Transport transport) {
    this.transport = transport;
    addRequirements(transport);

    this.up = up;
    this.down = down;
  }


  @Override
  public void initialize() {}

  
  @Override
  public void execute() {
    if(up.getAsBoolean()) {
      transport.setFeeder(-1);
    }
    else if(down.getAsBoolean()) {
      transport.setFeeder(1);
    }
    else if(!up.getAsBoolean() && !down.getAsBoolean()) {

      if(transport.getBeamBreak()) {
        transport.resetEncoderDelta();
        transport.setFeeder(-1);
        sensorWasTriggered = true;
      }
      else if(sensorWasTriggered) {
        if(transport.getEncoderDelta() > 999) { // TODO: get transport encoder values
          transport.setFeeder(0);
          transport.resetEncoderDelta();
          sensorWasTriggered = false;
        }
      else {
        transport.setFeeder(0);
      }

      }
    else {
      transport.setFeeder(0);
    }
    } // end of main if statement
  } // end of execute()

  
  @Override
  public void end(boolean interrupted) {
    transport.setFeeder(0);
  }

  
  @Override
  public boolean isFinished() {
    return false;
  }
}
