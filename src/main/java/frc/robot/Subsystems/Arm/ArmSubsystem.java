package frc.robot.Subsystems.Arm;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private final SparkMax intakeSpark = new SparkMax(ArmConstants.kIntakeSparkMaxCanId,
        ArmConstants.kIntakeMotorType);
    private final SparkMax rotationSpark = new SparkMax(ArmConstants.kRotationSparkMaxId,
        ArmConstants.kRotationMotorType);

    private final Encoder encoder = new Encoder(
        ArmConstants.kEncoderChannels[0], 
        ArmConstants.kEncoderChannels[1],
        ArmConstants.kEncoderReversed);

    

    public ArmSubsystem() {}

    @Override
    public void periodic(){
    }

    @Override
    public void simulationPeriodic(){
    }

    public void setSparkMaxConfig(){

    }
}
