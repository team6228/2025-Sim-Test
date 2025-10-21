package frc.robot.Subsystems.Arm;

import com.revrobotics.spark.SparkLowLevel.MotorType;

public final class ArmConstants {
    public static final int kIntakeSparkMaxCanId = 1;
    public static final int kRotationSparkMaxId = 2;

    public static final MotorType kIntakeMotorType = MotorType.kBrushless;
    public static final MotorType kRotationMotorType = MotorType.kBrushless;

    public static final int[] kEncoderChannels = {6,7};
    public static final boolean kEncoderReversed = false;

    public static final double kEncoderCPR = 1024;
    public static final double kDistancePerPulse = 0.0;
}
