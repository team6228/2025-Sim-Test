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
    public static final double kArmReduction = 50/14;
    public static final double kArmLength = .35;
    public static final double kArmMass = 1;
    public static final double kMinAngleRads = Units.degreesToRadians(-65);
    public static final double kMaxAngleRads = Units.degreesToRadians(105);

    //Encoders
    public static final int[] kEncoderChannels = {12,13};
    public static final boolean kEncoderReversed = true;

    public static final double kEncoderCPR = 1024;
    public static final double kDistancePerPulse = 2.0 * Math.PI / (kEncoderCPR * 4);

    //Controllers
    //40
    public static final double kP = 20.0;
    public static final double kI = 0.00;
    public static final double kD = 1.0;

    //Old values
    /* 
    public static final double kMaxVelocity = Units.degreesToRadians(155.0);
    public static final double kMaxAcceleration = Units.degreesToRadians(573.0);

    public static final double kS = 0.50;
    public static final double kG = 0.67;
    public static final double kV = 4.04;
    public static final double kA = 0.036;
    */

    //New values
    
    public static final double kMaxVelocity = Units.degreesToRadians(166.429); 
    public static final double kMaxAcceleration = Units.degreesToRadians(573.405); 

    public static final double kS = 0.20;    
    public static final double kG = 0.7085; 
    public static final double kV = 0.0721; 
    public static final double kA = 0.0168; 
    
}
