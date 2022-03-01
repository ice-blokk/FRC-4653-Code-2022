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
    public static final int INTAKE_ADDRESS = 99, // talon
                            ARM_ADDRESS = 99, // talon
                            INTAKE_OUT_HALLEFFECT_ADDRESS = 99, // hall effect sensor
                            INTAKE_IN_HALLEFFECT_ADDRESS = 99; // hall effect sensor

    public static final double kIntakeArmOutSoftLimit = 999,
                               kIntakeArmInSoftLimit = 999;

    //Shooter Constants
    public static final int LEAD_SHOOTER_ADDRESS = 99,
                            FOLLOWER_SHOOTER_ADDRESS = 99,
                            HOOD_ADJUSTER_ADDRESS = 99;

    public static final double kShooterP = 0.0005,
                               kShooterF = 0.0002148,
                               kShooterI = 0.00001; 

    public static final double kLimelightHeight = 1, // inches
                               kTargetHeight = 1, // inches
                               kLimelightAngle = 999; // degrees, based from the horizontal

                                

    // Turret constants
    public static final int TURRET_ADDRESS = 99, // talon
                            TURRET_FWD_HALL_EFFECT = 99,
                            TURRET_REV_HALL_EFFECT = 99,

                            kTurretSlotIdx = 0,
                            kTurretPIDLoopIdx = 0;
 
    public static final double  kTurretP = 99,
                                kTurretI = 99,
                                kTurretD = 99,
                                kTurretF = 99,

                                kSoftMaxTurretAngle = 99,
                                kSoftMinTurretAngle = 99,

                                kTurretSpurGearRatio = 149.0 / 10.0, // ratio of gear attached to turret to gear on motor
                                kTurretMotorGearRatio = 45.0 / 1.0, // ratio of motor with gearbox
                                kVersaPlanetaryEncoderCountsPerRevolution = 1024, // CPS of encoder on motor
                                kTurretRotationsPerTick = (1.0 / kVersaPlanetaryEncoderCountsPerRevolution) * (1.0 / kTurretMotorGearRatio) / (1.0 / kTurretSpurGearRatio); 
                                // i have no idea of the above calculation is at all correct

    // Transport constants
    public static final int TRANSPORT_ADDRESS = 99,
                            TRANSPORT_BEAM_BREAK_ADDRESS = 99;

    


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
