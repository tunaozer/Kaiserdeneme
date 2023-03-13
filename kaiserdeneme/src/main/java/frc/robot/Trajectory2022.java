package frc.robot;

import java.util.List;

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
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.subsystems.DriveTrain;

public class Trajectory2022 {
	private DriveTrain m_drive;
    public Trajectory[] auto1 = new Trajectory[2];

	public Trajectory2022(DriveTrain drive){
        m_drive = drive;
        var autoVoltageConstraint =
            new DifferentialDriveVoltageConstraint(
                new SimpleMotorFeedforward(
                    TrajectoryConstants.ksVolts,
                    TrajectoryConstants.kvVoltSecondsPerMeter,
                    TrajectoryConstants.kaVoltSecondsSquaredPerMeter
                ),
                TrajectoryConstants.kDriveKinematics,
                TrajectoryConstants.kMaxAutoVoltage
            );
        TrajectoryConfig configForward =
            new TrajectoryConfig(
                TrajectoryConstants.kMaxSpeedMetersPerSecond,
                TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared
            )
            .setKinematics(TrajectoryConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);
        configForward.setReversed(false);
        TrajectoryConfig configBackward =
            new TrajectoryConfig(
                TrajectoryConstants.kMaxSpeedMetersPerSecond,
                TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared
            )
            .setKinematics(TrajectoryConstants.kDriveKinematics)
            .addConstraint(autoVoltageConstraint);
        configBackward.setReversed(true);
        auto1[0] = TrajectoryGenerator.generateTrajectory(
            List.of(
                new Pose2d(0, 0, new Rotation2d(0)),
                new Pose2d(5, 1, new Rotation2d(Math.toRadians(0)))),
                configForward
        );
        auto1[1] = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-4, 0, new Rotation2d(0)),
            List.of(
                new Translation2d(-2, -0.5)),
                new Pose2d(-0.5, -1, new Rotation2d(Math.toRadians(0))),
                configForward
        );
    }
    public RamseteCommand getRamsete(Trajectory trajectory){
        return new RamseteCommand(
            trajectory,
            m_drive::getPose,
            new RamseteController(
                TrajectoryConstants.kRamseteB,
                TrajectoryConstants.kRamseteZeta
            ),
            new SimpleMotorFeedforward(
                TrajectoryConstants.ksVolts,
                TrajectoryConstants.kvVoltSecondsPerMeter,
                TrajectoryConstants.kaVoltSecondsSquaredPerMeter
            ),
            TrajectoryConstants.kDriveKinematics,
            m_drive::getWheelSpeeds,
            new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
            new PIDController(TrajectoryConstants.kPDriveVel, 0, 0),
            m_drive::tankDriveVolts,
            m_drive
        );
    }
}
