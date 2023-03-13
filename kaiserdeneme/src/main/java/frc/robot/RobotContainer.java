package frc.robot;


import edu.wpi.first.wpilibj.PS4Controller;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.DriveAuto;

import frc.robot.commands.TeleopDriveCommand;

import frc.robot.subsystems.DriveTrain;


public class RobotContainer {
	/*
	** Drive Train +
	** Intake + 
	** Shooter +

	** Climber
	** Climber Angle
	** Addressable Led
	*/
	private DriveTrain driveTrain = new DriveTrain();

	public PS4Controller driverJoystick = new PS4Controller(Constants.JoystickConstants.driverPort);

	public DriveAuto drive = new DriveAuto(driveTrain);
	public RobotContainer() {
		driveTrain.setDefaultCommand(new TeleopDriveCommand(driverJoystick, driveTrain));
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		//new JoystickButton(driverJoystick, 1).whileHeld(new IntakeCom(intakeSub));

		//new JoystickButton(driverJoystick, 3).whileHeld(new ShooterCom(shooter)); 
		//new JoystickButton(driverJoystick, 12).whileHeld(new ShooterSolenoidCom(shooter)); 
		//new POVButton(driverJoystick, 180).whileHeld(new IndexCom(shooter));
		new JoystickButton(driverJoystick, 8).onTrue(Commands.runOnce(() -> driveTrain.changeSlowMode(),driveTrain));
		//new JoystickButton(driverJoystick, 7).whileHeld(new ClimbCom(climb, true)); 
		//new JoystickButton(driverJoystick, 8).whileHeld(new ClimbCom(climb, false));  
		
		//new JoystickButton(driverJoystick, 7).whileHeld(new ClimbCom(climb, true)); 
		//new JoystickButton(driverJoystick, 8).whileHeld(new ClimbCom(climb, false)); 

		//new POVButton(driverJoystick, 270).whenPressed(() -> intakeSub.disablecomp());
		//new POVButton(driverJoystick, 90).whenPressed(() -> intakeSub.activeComp());
	}

	public Command getAutonomousCommand() {
		return drive;
		//return null;
	}
}


