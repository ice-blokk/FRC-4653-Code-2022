// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  // Basic drivetrain stuff
  private final CANSparkMax frontLeft, frontRight, backLeft, backRight;
  private final MotorControllerGroup left, right;
  private final DifferentialDrive drive;

  private int invert = 1;

  private final AHRS ahrs;

  // Odometry & trajectory
  private final DifferentialDriveOdometry odometry;

	private RelativeEncoder leftEncoder;
	private RelativeEncoder rightEncoder;

	DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(Constants.kTrackwidthMeters);

	SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter);


	DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(feedforward, kDriveKinematics, 10);
	
  TrajectoryConfig config = new TrajectoryConfig(
		Constants.kMaxSpeedMetersPerSecond, 
		Constants.kMaxAccelerationMetersPerSecondSquared)
		.setKinematics(kDriveKinematics)
		.addConstraint(autoVoltageConstraint);

  public Drivetrain() {
    frontLeft = new CANSparkMax(Constants.DRIVE_FRONT_LEFT_ADDRESS, MotorType.kBrushless);
    frontRight = new CANSparkMax(Constants.DRIVE_FRONT_RIGHT_ADDRESS, MotorType.kBrushless);
    backLeft = new CANSparkMax(Constants.DRIVE_BACK_LEFT_ADDRESS, MotorType.kBrushless);
    backRight = new CANSparkMax(Constants.DRIVE_BACK_RIGHT_ADDRESS, MotorType.kBrushless);

    frontLeft.restoreFactoryDefaults();
    frontRight.restoreFactoryDefaults();
    backLeft.restoreFactoryDefaults();
    backRight.restoreFactoryDefaults();

    frontLeft.setIdleMode(IdleMode.kBrake);
    frontRight.setIdleMode(IdleMode.kBrake);
    backLeft.setIdleMode(IdleMode.kBrake);
    backRight.setIdleMode(IdleMode.kBrake);

    left = new MotorControllerGroup(frontLeft, backLeft);
    right = new MotorControllerGroup(frontRight, backRight);

    drive = new DifferentialDrive(left, right);

    ahrs = new AHRS(SPI.Port.kMXP);

    odometry = new DifferentialDriveOdometry(getHeading());

    leftEncoder = frontLeft.getEncoder();
	rightEncoder = frontRight.getEncoder();

  }

  public void tankDrive(double leftSpeed, double rightSpeed, boolean sqareInputs) {
		drive.tankDrive(leftSpeed, rightSpeed, sqareInputs);
	}

	public void tankDrive(double leftSpeed, double rightSpeed) {
		drive.tankDrive(leftSpeed, rightSpeed);
	}

	public void arcadeDrive(double speed, double rotate) {
		drive.arcadeDrive(speed, rotate);
	}
	public void arcadeDrive(double speed, double rotate, boolean isInverted) {
		if(isInverted) {invert *= -1;}
		drive.arcadeDrive(speed * invert, rotate);
	}

  // Gyro methods
  double gyroOffset = 0;
	public double getAngle() {
		return ahrs.getAngle() - gyroOffset;
	}
	public void resetAngle() {
		gyroOffset = ahrs.getAngle();
	}

  // Odometry Methods
	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(
			// divide by gear ratio, multiply by wheel circumference (converted to meters), 
			// convert to per second by dividing by 60 (since getVelocity gives RPM)
			leftEncoder.getVelocity() / 7.31 * Math.PI * Units.inchesToMeters(6.0) / 60, 
			rightEncoder.getVelocity() / 7.31 * Math.PI * Units.inchesToMeters(6.0) / 60
			);
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		left.setVoltage(leftVolts);
		right.setVoltage(-rightVolts);
		drive.feed();
	}

	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		odometry.resetPosition(pose, ahrs.getRotation2d());
	}

	public void setMaxOutput(double maxOutput) {
		drive.setMaxOutput(maxOutput);
	}

	public void zeroHeading() {
		ahrs.reset();
	}

	public Rotation2d getHeading() {
		return Rotation2d.fromDegrees(-ahrs.getAngle());
	}

	public double getTurnRate() {
		return -ahrs.getRate();
	}
	
  public void resetEncoders() {
		frontLeft.getEncoder().setPosition(0);
		frontRight.getEncoder().setPosition(0);
		backLeft.getEncoder().setPosition(0);
		backRight.getEncoder().setPosition(0);
	}

  // TODO: get gear ratio of current drivetrain, put it in constants, change 7.31 in methods to constant

	// Getting distance from encoder values
	// getPosition() returns the amount of turns of the shaft, so to get distance (in inches)
	// divide shaft turns by gear ratio and multiply by the circumferance (pi times diameter of the wheels)
	// then convert from inches to meters
	public double getLeftEncoderDistance() {
		return Units.inchesToMeters((leftEncoder.getPosition() / 7.31) * Math.PI * 6.0);
	}

	public double getRightEncoderDistance() {
		return Units.inchesToMeters((-rightEncoder.getPosition() / 7.31) * Math.PI * 6.0);
	}

	// Methods for creating trajectories
	public SimpleMotorFeedforward getFeedforward() {
		return feedforward;
	}
 
	public DifferentialDriveVoltageConstraint getAutoVoltageConstrain() {
		return autoVoltageConstraint;
	}

	public TrajectoryConfig getConfig() {
		return config;
	}

	public RamseteController getRamseteController() {
		return new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta);
	}

	public PIDController getLeftPID() {
		return new PIDController(Constants.kPDriveVel, 0, 0);
	}

	public PIDController getRightPID() {
		return new PIDController(Constants.kPDriveVel, 0, 0);
	}

	public DifferentialDriveKinematics getDriveKinematics() {
		return kDriveKinematics;
	}

	public Pose2d getCurrentPose() {
		return odometry.getPoseMeters();
	}

  @Override
  public void periodic() {
	  SmartDashboard.putBoolean("Drivetrain Inverted", invert == -1);
	  
	  SmartDashboard.putNumber("Odometry X", odometry.getPoseMeters().getX());
	  SmartDashboard.putNumber("Odometry Y", odometry.getPoseMeters().getY());
  }
}
