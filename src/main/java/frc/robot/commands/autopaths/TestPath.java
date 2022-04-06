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
            // new Pose2d(),
            // List.of(
            //   //new Translation2d(1.4, 0) //distance of about 28.89 inches
            //   new Translation2d(.5, 0)
            //   //new Translation2d(1.5, 1)
              

            // ),
            // //new Pose2d(.5, -2.4, new Rotation2d(Math.toRadians(-135))),
            // new Pose2d(3.0, 0, new Rotation2d()),
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(
                new Translation2d(1, 1),
                new Translation2d(2, -1)
            ),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(3, 0, new Rotation2d(0)),
            drivetrain.getConfig(.5, .5, false)
          );

        return testPath;
    }
    
}
