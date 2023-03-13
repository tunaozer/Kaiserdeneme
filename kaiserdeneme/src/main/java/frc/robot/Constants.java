package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public final class Constants {
    public static final class TrajectoryConstants{
        public static final double ksVolts = 1.4;
        public static final double kvVoltSecondsPerMeter = 3.4; 
        public static final double kaVoltSecondsSquaredPerMeter = 0.1;
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double kPDriveVel = 1.88;
        public static final double kTrackwidthMeters = 0.63;
        public static final double kMaxAutoVoltage = 10;
        public static final boolean kGyroReversed = false;
        public static final double kEncoderCPR = 4096;
        public static final double kWheelDiameterMeters = 0.1524;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
    }
    public static final class JoystickConstants{
        // Port
        public static final int driverPort = 0;
        public static final int coPort = 1;
    }
    public static final class DriveTrainConstants{
        // Port Constants
        public static final int leftMasterPort = 2;
        public static final int leftSlavePort = 3;
        public static final int rightMasterPort = 12;
        public static final int rightSlavePort = 13;
        // Invert Constants
        public static final InvertType leftMasterInvert = InvertType.None;
        public static final InvertType leftSlaveInvert = InvertType.None;
        public static final InvertType rightMasterInvert = InvertType.None;
        public static final InvertType rightSlaveInvert = InvertType.None;
        // Brake Constants
        public static final NeutralMode leftMasterBrake = NeutralMode.Brake;
        public static final NeutralMode leftSlaveBrake = NeutralMode.Brake;
        public static final NeutralMode rightMasterBrake = NeutralMode.Brake;
        public static final NeutralMode rightSlaveBrake = NeutralMode.Brake;
        // Safety Constants
        public static final boolean leftMasterSafety = false;
        public static final boolean leftSlaveSafety = false;
        public static final boolean rightMasterSafety = false;
        public static final boolean rightSlaveSafety = false;
        public static final boolean driveSafety = false;
        // Speed Constants
        public static final double driveLowSpeed = 0.35;
        public static final double driveDefaultSpeed = 0.6;
        public static final double driveHighSpeed = 1;
    }
    public static final class IntakeConstants{
        // Port Constants
        public static final int intakePort = 6;
        // Solenoid Constants
        public static final int intakeSolenoidPort = 2;
        public static final int intakeSolenoidPort2 = 3;
        // Invert Constants
        public static final InvertType intakeInvert = InvertType.None;
        // Brake Constants
        public static final NeutralMode intakeBrake = NeutralMode.Brake;
        // Safety Constants
        public static final boolean intakeSafety = false;
        // Speed Constants
        public static final double intakeSpeed = -1;
        public static final double intakeInvertSpeed = 1;
    }
    public static final class ShooterConstants{
        // Port Constants
        public static final int shooterPort = 62;
        public static final int shooterSolenoidPort = 5;
        // Speed Constants
        public static final double shooterSpeed = -0.9;
    }
    public static final class IndexConstants{
        // Port Constants
        public static final int indexerPort = 5;
        // Speed Constants
        public static final double indexerSpeed = 0.5;
    }
    public static final class ClimbConstants{
        // Port Constants
        public static final int climbPort = 7;
        public static final int climbSolenoidPort = 0;
        public static final int climbSolenoidPort2 = 1;
        // Speed Constants 
        public static final double climbSpeed = 1;
        public static final double invertClimbSpeed = -1;
        // 

    }
}
