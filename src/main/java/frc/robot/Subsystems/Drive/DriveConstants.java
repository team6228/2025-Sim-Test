package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class DriveConstants {
    public static final double kFwdCap = 1.0;
    public static final double kFwdRot = -1.0;

    public static final double kRotCap = 1.0;
    public static final double kRotRot = 1.0;

    public static final int kRightMotorChannel = 1;
    public static final boolean kRightMotorReversed = true;

    public static final int kLeftMotorChannel = 0;
    public static final boolean kLeftMotorReversed = false;

    public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
    public static final double kDriveGearing = 8;
    public static final double kMoiOfRobot = 7.5;
    public static final double kMassOfRobot = 60.0;
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    public static final double kTrackWidthMeters = 0.69;

    public static final int[] kRightEncoderChannels = {2,3};
    public static final boolean kRightEncoderReversed = false;

    public static final int[] kLeftEncoderChannels = {4,5};
    public static final boolean kLeftEncoderReversed = true;

    public static final double kEncoderCPR = 1024;
    public static final double kDistancePerPulse = (kTrackWidthMeters / Math.PI) / kEncoderCPR;

    public static final boolean kGyroReversed = true;
}