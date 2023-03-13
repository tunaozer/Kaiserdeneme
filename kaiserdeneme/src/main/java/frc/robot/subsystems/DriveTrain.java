// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.TrajectoryConstants;

public class DriveTrain extends SubsystemBase {
	enum SpeedMode{
		Low,
		Default,
		High
	}
	//private SpeedMode speedMode = SpeedMode.Default;

	private WPI_TalonSRX leftMaster;
	private WPI_TalonSRX leftSlave;
	private WPI_TalonSRX rightMaster;
	private WPI_TalonSRX rightSlave;

	private DifferentialDrive drive;

	public final DifferentialDriveOdometry m_odometry;
    public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
	private final Field2d m_field = new Field2d();
	public DriveTrain() {

		leftMaster = new WPI_TalonSRX(Constants.DriveTrainConstants.leftMasterPort);
		leftSlave = new WPI_TalonSRX(Constants.DriveTrainConstants.leftSlavePort);
		rightMaster = new WPI_TalonSRX(Constants.DriveTrainConstants.rightMasterPort);
		rightSlave = new WPI_TalonSRX(Constants.DriveTrainConstants.rightSlavePort);

		leftSlave.follow(leftMaster);
		rightSlave.follow(rightMaster);

		drive = new DifferentialDrive(leftMaster, rightMaster);
		leftMaster.setInverted(Constants.DriveTrainConstants.leftMasterInvert);
		leftSlave.setInverted(Constants.DriveTrainConstants.leftSlaveInvert);
		rightMaster.setInverted(Constants.DriveTrainConstants.rightMasterInvert);
		rightSlave.setInverted(Constants.DriveTrainConstants.rightSlaveInvert);
		leftMaster.setNeutralMode(Constants.DriveTrainConstants.leftMasterBrake);
		leftSlave.setNeutralMode(Constants.DriveTrainConstants.leftSlaveBrake);
		rightMaster.setNeutralMode(Constants.DriveTrainConstants.rightMasterBrake);
		rightSlave.setNeutralMode(Constants.DriveTrainConstants.rightSlaveBrake);
		leftMaster.setSafetyEnabled(Constants.DriveTrainConstants.leftMasterSafety);
		leftSlave.setSafetyEnabled(Constants.DriveTrainConstants.leftSlaveSafety);
		rightMaster.setSafetyEnabled(Constants.DriveTrainConstants.rightMasterSafety);
		rightSlave.setSafetyEnabled(Constants.DriveTrainConstants.rightSlaveSafety);
		drive.setSafetyEnabled(Constants.DriveTrainConstants.driveSafety);
		leftMaster.setSensorPhase(true);
		rightMaster.setSensorPhase(true);

		m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()), getLeftEncoderDistance(), getRightEncoderDistance());
        SmartDashboard.putData("Field", m_field);
		zeroHeading();
        resetEncoders();
	}


	@Override
	public void periodic() {
		
	}
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}
	public double getHeading() {
		return Math.IEEEremainder(m_gyro.getAngle(), 360) * (TrajectoryConstants.kGyroReversed ? -1.0 : 1.0);
    }
	public void zeroHeading() {
        m_gyro.reset();
    }
    public void resetEncoders() {
        leftMaster.setSelectedSensorPosition(0);
        rightMaster.setSelectedSensorPosition(0);
    }
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(
			10.0
			* leftMaster.getSelectedSensorVelocity()
			* (1.0 / TrajectoryConstants.kEncoderCPR)
			* (Math.PI * TrajectoryConstants.kWheelDiameterMeters),
			10.0
			* rightMaster.getSelectedSensorVelocity()
			* (1.0 / TrajectoryConstants.kEncoderCPR)
			* (-Math.PI * TrajectoryConstants.kWheelDiameterMeters));
    }
	public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMaster.setVoltage(leftVolts);
        rightMaster.setVoltage(-rightVolts);
        drive.feed();
    }
	boolean isSlowMode = false;
	public void arcadeDrive(double speed, double rotation, boolean temp_squ){
		
		double xSpeed = speed;
		double zRotation = rotation;
	
		if (xSpeed == 1) {
		  zRotation *= 1;
		} else
		  zRotation *= 1;
	
		if (isSlowMode) {
		  xSpeed *= 0.3;
		  zRotation *= 0.6;
		}
	
		xSpeed = Deadband(xSpeed);
		zRotation = Deadband(zRotation);
	
		leftMaster.set(ControlMode.PercentOutput, xSpeed, DemandType.ArbitraryFeedForward, +zRotation);
		rightMaster.set(ControlMode.PercentOutput, xSpeed, DemandType.ArbitraryFeedForward, -zRotation);
	
	}
	public double getRightEncoderDistance() {
        return rightSlave.getSelectedSensorPosition()
                * (1.0 / TrajectoryConstants.kEncoderCPR)
                * (Math.PI * TrajectoryConstants.kWheelDiameterMeters);
    }

    public double getLeftEncoderDistance() {
        return leftSlave.getSelectedSensorPosition()
                * (1.0 / TrajectoryConstants.kEncoderCPR)
                * (-Math.PI * TrajectoryConstants.kWheelDiameterMeters);
    }
	double Deadband(double i){
		return ((i >= +0.09) ? i : ((i <= -0.09) ? i : 0));
	}
	public void changeSlowMode(){
		if (isSlowMode){
			isSlowMode = false;
		}else{
			isSlowMode = true;
		}
	}
}
