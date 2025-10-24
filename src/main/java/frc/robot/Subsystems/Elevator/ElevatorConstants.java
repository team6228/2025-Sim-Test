package frc.robot.Subsystems.Elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public final class ElevatorConstants {
    public static final int kFollowerSparkMacCanId = 3;
    public static final int kFollowerSparkMaxPort = 9;
    public static final MotorType kFollowerMotorType = MotorType.kBrushless;
    public static final boolean kFollowerMotorInvert = false;
    
    public static final int kLeaderSparkMaxCanId = 4;
    public static final int kLeaderSparkMaxPort = 10;
    public static final MotorType kLeaderMotorType = MotorType.kBrushless;
    public static final boolean kLeaderMotorInvert = true;

    //Elevator sim stuff
    public static final double kMinElevatorHeightMeters = 0.0;
    public static final double kMaxElevatorHeightMeters = 1.25;

    public static final DCMotor kGearBox = DCMotor.getNEO(2);
    public static final double kElevatorGearing = 10.0;
    public static final double kCarriageMass = 4.0;
    public static final double kElevatorDrumRadius = Units.inchesToMeters(2.0);
    /*[TODO] Elevator Sim may have some more parameters */

    /*Encoder */
    public static final int[] kEncoderChannels = {8,9};
    public static final boolean kEncoderReversed = true;

    public static final double kEncoderCPR = 1024;
    public static final double kDistancePerPulse = 2.0 * Math.PI * kElevatorDrumRadius / kEncoderCPR * 4;

    public static final double kFeederStationPosition = 0.50;
    public static final double kProccescorPosition = 0.0;
    public static final double kL1Position = 0.20;
    public static final double kL2Position = 0.70;
    public static final double kL3Position = 1.00;
    public static final double kL4Position = 1.50;

    /*General config */
    public static final int kSmartCurrent = 40;
    public static final IdleMode kIdleMode = IdleMode.kBrake;

    /*Encoder config */
    public static final double kPositionConversationFactor = 1000;
    public static final double kVelocityConversationFactor = 1000;

    /*Feedback loop config */
    /*[TODO] dosya icindeki sensor bunun yerine kullanilabilir */
    public static final FeedbackSensor kFeedbackSensor = FeedbackSensor.kPrimaryEncoder;
    public static final double kP = 20.0;
    public static final double kI = 0.01;
    public static final double kD = 2.0;
    public static final double kIZone = 0.01;

    public static final double kS = 0.0;
    public static final double kG = 0.25;
    public static final double kV = 0.25;
    public static final double kA = 0.0;
}
