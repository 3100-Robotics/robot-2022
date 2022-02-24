/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.autonomous;

import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Drivetrain.Drive;

public class PathGenerator extends CommandBase {

    private static Drive m_robotDrive;

    public PathGenerator(Drive subsystem) {

        m_robotDrive = subsystem;
        addRequirements(m_robotDrive);

    }

    public static SequentialCommandGroup generateTrajectory(String pathName) {

        // An example trajectory to follow. All units in meters.
        // Name of the path, maxVelocity, maxAcceleration
        Trajectory exampleTrajectory = PathPlanner.loadPath("Testing A Curve", 8, 5);

    //     var autoVoltageConstraint =
    //     new DifferentialDriveVoltageConstraint(
    //         new SimpleMotorFeedforward(
    //             TrajectoryConstants.ksVolts,
    //             TrajectoryConstants.kvVoltSecondsPerMeter,
    //             TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
    //             TrajectoryConstants.kDriveKinematics,
    //         10);

    // // Create config for trajectory
    // TrajectoryConfig config =
    //     new TrajectoryConfig(
    //         TrajectoryConstants.kMaxSpeedMetersPerSecond,
    //         TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared)
    //         // Add kinematics to ensure max speed is actually obeyed
    //         .setKinematics(TrajectoryConstants.kDriveKinematics)
    //         // Apply the voltage constraint
    //         .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory =
    //     TrajectoryGenerator.generateTrajectory(
    //         // Start at the origin facing the +X direction
    //         new Pose2d(0, 0, new Rotation2d(0)),
    //         // Pass through these two interior waypoints, making an 's' curve path
    //         List.of(new Translation2d(1, 1), new Translation2d(2, 0)),
    //         // End 3 meters straight ahead of where we started, facing forward
    //         new Pose2d(3, 0, new Rotation2d(0)),
    //         // Pass config
    //         config);

        RamseteCommand ramseteCommand = new RamseteCommand(
                exampleTrajectory,
                m_robotDrive::getPose,
                new RamseteController(TrajectoryConstants.kRamseteB, TrajectoryConstants.kRamseteZeta),
                new SimpleMotorFeedforward(
                    TrajectoryConstants.ksVolts,
                    TrajectoryConstants.kvVoltSecondsPerMeter,
                    TrajectoryConstants.kaVoltSecondsSquaredPerMeter),
                    TrajectoryConstants.kDriveKinematics,
                m_robotDrive::getWheelSpeeds,
                new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
                new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
                // RamseteCommand passes volts to the callback
                m_robotDrive::tankDriveVolts,
                m_robotDrive);

        // Reset odometry to the starting pose of the trajectory.
        m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

        // Run path following command, then stop at the end.
        return ramseteCommand.andThen(() -> m_robotDrive.tankDriveVolts(0, 0));

    }

}