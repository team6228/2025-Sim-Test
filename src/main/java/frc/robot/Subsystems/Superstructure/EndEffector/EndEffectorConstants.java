package frc.robot.Subsystems.Superstructure.EndEffector;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class EndEffectorConstants {
    public static final int kIntakeSparkMaxCanId = 1;
    public static final int kRotationSparkMaxCanId = 1;

    public static final int kIntakeSparkMaxPort = 10;
    public static final int kRotationSparkMaxPort = 11;

    public static final MotorType kIntakeMotorType = MotorType.kBrushless;
    public static final MotorType kRotationMotorType = MotorType.kBrushless;

    //Arm sim
    public static final DCMotor kArmGearBox = DCMotor.getNEO(1);
    public static final double kArmReduction = 200;
    public static final double kArmLength = Units.inchesToMeters(30);
    public static final double kArmMass = 8.0;
    public static final double kMinAngleRads = Units.degreesToRadians(0);
    public static final double kMaxAngleRads = Units.degreesToRadians(360);

    //Encoders
    public static final int[] kEncoderChannels = {12,13};
    public static final boolean kEncoderReversed = false;

    public static final double kEncoderCPR = 1024;
    public static final double kDistancePerPulse = 2.0 * Math.PI / (kEncoderCPR * 4);

    //Controllers
    public static final double kP = 40.0;
    public static final double kI = 0.01;
    public static final double kD = 5.0;

    public static final double kMaxVelocity = Units.degreesToRadians(155.0);
    public static final double kMaxAcceleration = Units.degreesToRadians(573.0);

    public static final double kS = 0.50;
    public static final double kG = 0.67;
    public static final double kV = 4.04;
    public static final double kA = 0.036;
}
