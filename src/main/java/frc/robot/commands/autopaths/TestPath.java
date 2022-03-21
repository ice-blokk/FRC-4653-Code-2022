package frc.robot.commands.autopaths;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.subsystems.Drivetrain;

public final class TestPath {
    public static Trajectory getTraj(Drivetrain drivetrain) {
        Trajectory testPath = TrajectoryGenerator.generateTrajectory(
            drivetrain.getCurrentPose(),
            List.of(
              new Translation2d(1.4, 0)

            ),
            new Pose2d(.5, -2.4, new Rotation2d(Math.toRadians(-135))),
            drivetrain.getSlowConfig()
          );

        return testPath;
    }
    
}
