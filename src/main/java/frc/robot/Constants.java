// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    // Drive motor ports (all sparks)
    public static final int DRIVE_FRONT_LEFT_ADDRESS = 1,
                            DRIVE_FRONT_RIGHT_ADDRESS = 2,
                            DRIVE_BACK_LEFT_ADDRESS = 3, 
                            DRIVE_BACK_RIGHT_ADDRESS = 4;

    // Intake constants
    public static final int INTAKE_ADDRESS = 11, // talon
                            ARM_ADDRESS = 7; // spark

    public static final double kIntakeArmOutSoftLimit = -16000,
                               kIntakeArmInSoftLimit = 10;

    //Shooter Constants
    public static final int LEAD_SHOOTER_ADDRESS = 5, // spark, facing the front, the motor on the left
                            FOLLOWER_SHOOTER_ADDRESS = 6, // spark, facing the front, motor is on the right
                            HOOD_ADJUSTER_ADDRESS = 9; // rev servo

    public static final double kShooterP = 0.00008,
                               kShooterI = 0.0000001,
                               //kShooterD = 0.001,
                               kShooterF = 0.000175; // 148;
                               

    public static final double kLimelightHeight = 50, // inches (total is 4ft 2 in)
                               kTargetHeight = 104, // inches (total is 8 ft 8 in)
                               kLimelightAngle = 35; // degrees, based from the horizontal

                                

    // Turret constants
    public static final int TURRET_ADDRESS = 9, // talon
                            TURRET_FWD_LIMIT_SWITCH = 1, // DIO
                            TURRET_REV_LIMIT_SWITCH = 2, // DIO

                            kTurretSlotIdx = 0,
                            kTurretPIDLoopIdx = 0;
 
    public static final double  kTurretP = 99,
                                kTurretI = 99,
                                kTurretD = 99,
                                kTurretF = 99,

                                kSoftMaxTurretAngle = 9999,
                                kSoftMinTurretAngle = 9999,

                                kTurretSpurGearRatio = 149.0 / 10.0, // ratio of gear attached to turret to gear on motor
                                kTurretMotorGearRatio = 45.0 / 1.0, // ratio of motor with gearbox
                                kVersaPlanetaryEncoderCountsPerRevolution = 1024, // CPR of encoder on motor
                                kTurretRotationsPerTick = .00001965;
                                //kTurretRotationsPerTick = (1.0 / kVersaPlanetaryEncoderCountsPerRevolution) * (1.0 / kTurretMotorGearRatio) / (1.0 / kTurretSpurGearRatio); 
                                // i have no idea of the above calculation is at all correct

    // Transport constants
    public static final int TRANSPORT_ADDRESS = 10, // talon
                            TRANSPORT_BEAM_BREAK_ADDRESS = 0; // DIO

    // Climber constants
    public static final int CLIMBER_LEADER_ADDRESS = 15,
                            CLIMBER_FOLLOWER_ADDRESS = 14;

    //Climber PID/FF Values
    public static final double leadClimberP = 0.1,
                               leadClimberI = 0.00001,
                               leadClimberD = 0.00001,
                               leadClimberF = 0.00001,

                               followerClimberP = 0.1,
                               followerClimberI = 0.00001,
                               followerClimberD = 0.00001,
                               followerClimberF = 0.00001;


    // Odometry + Trajectory Constants

    // Characterization constants gotten from team 5910 (they seem to work
    // better than the tool gotten from our testing)
    public static final double ksVolts = 0.268;
    public static final double kvVoltSecondsPerMeter = 1.89;
    public static final double kaVoltSecondsSquaredPerMeter = .0243; // this is the value from team 5190

    // P val gotten from characterization
    public static final double kPDriveVel = 0.205;

    public static final double kTrackwidthMeters = 0.5588;

	public static final double kMaxSpeedMetersPerSecond = 1.5;
	public static final double kMaxAccelerationMetersPerSecondSquared = 1.7;
	
	public static final double kRamseteB = 2;
	public static final double kRamseteZeta = 0.7;


}
