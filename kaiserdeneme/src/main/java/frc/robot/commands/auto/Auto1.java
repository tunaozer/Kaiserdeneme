package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Trajectory2022;
import frc.robot.subsystems.DriveTrain;
public class Auto1 extends SequentialCommandGroup {
	public Auto1(
		Trajectory2022 trajectory,
		DriveTrain drive
	)
	{
		super(
			trajectory
				.getRamsete(trajectory.auto1[0]),
			trajectory
				.getRamsete(trajectory.auto1[1])
				.andThen(() -> drive.tankDriveVolts(0, 0)),
			trajectory.getRamsete(trajectory.auto1[0])
		);
	}
}
