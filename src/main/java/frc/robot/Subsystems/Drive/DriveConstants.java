package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.system.plant.DCMotor;

public final class DriveConstants {
    public static final double kWheelSpeedCap = 1.0;

    //[TODO] Make sure of reverse values
    public static final int kLeftFrontMotorChannel = 0;
    public static final boolean kLeftFrontMotorReversed = true;

    public static final int kRightFrontMotorChannel = 1;
    public static final boolean kRightFrontMotorReversed = false;

    public static final int kLeftRearMotorChannel = 14;
    public static final boolean kLeftRearMotorReversed = true;

    public static final int kRightRearMotorChannel = 15;
    public static final boolean kRightRearMotorReversed = false;

    public static final double kMaxWheelSpeed = 3.0; //m/s

    public static final DCMotor kDriveGearbox = DCMotor.getCIM(2);
    public static final double kDriveGearing = 10;
    public static final double kMoiOfRobot = 4;
    public static final double kMassOfRobot = 51.5;
    public static final double kWheelDiameterMeters = 0.15;
    public static final double kTrackWidthMeters = 0.67;

    public static final int[] kLeftFrontEncoderChannels = {4,5};
    public static final boolean kLeftFrontEncoderReversed = true;

    public static final int[] kRightFrontEncoderChannels = {2,3};
    public static final boolean kRightFrontEncoderReversed = false;

    public static final int[] kLeftRearEncoderChannels = {16,17};
    public static final boolean kLeftRearEncoderReversed = true;

    public static final int[] kRightRearEncoderChannels = {20,21};
    public static final boolean kRightRearEncoderReversed = false;

    public static final double kEncoderCPR = 1024;
    public static final double kDistancePerPulse = (kTrackWidthMeters / Math.PI) / kEncoderCPR;

    public static final boolean kGyroReversed = true;
}