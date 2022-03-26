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
            new Pose2d(),
            List.of(
              //new Translation2d(1.4, 0) //distance of about 28.89 inches
              new Translation2d(1.0, 0)
              

            ),
            //new Pose2d(.5, -2.4, new Rotation2d(Math.toRadians(-135))),
            new Pose2d(3.0, 0, new Rotation2d()),
            drivetrain.getSlowConfig()
          );

        return testPath;
    }
    
}
