package frc.robot.Subsystems.Elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatorSubsystem extends SubsystemBase{
    private final SparkMaxConfig leaderConfig = new SparkMaxConfig();
    private final SparkMaxConfig followerConfig = new SparkMaxConfig();

    private final SparkMax followerSpark = new SparkMax(ElevatorConstants.kFollowerSparkMacCanId,
        ElevatorConstants.kFollowerMotorType);
    private final SparkMax leaderSpark = new SparkMax(ElevatorConstants.kLeaderSparkMaxCanId,
        ElevatorConstants.kLeaderMotorType);

    private final Encoder encoder = new Encoder(
        ElevatorConstants.kEncoderChannels[0],
        ElevatorConstants.kEncoderChannels[1],
        ElevatorConstants.kEncoderReversed);

    //[TODO] Feedback controllers
    //[TODO] May use pwm sim

    //[TODO] Check last parameters
    private final ElevatorSim elevatorSim = new ElevatorSim(
    ElevatorConstants.kGearBox,
    ElevatorConstants.kElevatorGearing,
    ElevatorConstants.kCarriageMass,
    ElevatorConstants.kElevatorDrumRadius,
    ElevatorConstants.kMinElevatorHeightMeters,
    ElevatorConstants.kMaxElevatorHeightMeters,
    true,
    0,
    0.01,
    0.0);

    private final EncoderSim encoderSim = new EncoderSim(encoder);

    private final Mechanism2d mech2d = new Mechanism2d(20,50);
    private final MechanismRoot2d rootMech2d = mech2d.getRoot("Elevator root", 10, 0);
    private final MechanismLigament2d elevatorMech2d = rootMech2d.append(
        new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90)
    );

    public ElevatorSubsystem() {
        encoder.reset();

        encoder.setDistancePerPulse(ElevatorConstants.kDistancePerPulse);

        setSparkMaxConfig();

        SmartDashboard.putData("Elevator",mech2d);
    }

    @Override
    public void periodic(){

    }

    @Override
    public void simulationPeriodic(){
        //[TODO] may use sim motor 
        elevatorSim.setInput(leaderSpark.get() * RobotController.getBatteryVoltage());

        elevatorSim.update(.02);

        encoderSim.setDistance(elevatorSim.getPositionMeters());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }

    public void reachGoal(double goal){
        //[TODO] implement controller

        double pidOutput = 0.0;
        double feedForward = 0.0;

        leaderSpark.setVoltage(pidOutput + feedForward);
    }

    public void setSparkMaxConfig(){
        leaderConfig
            .inverted(ElevatorConstants.kLeaderMotorInvert)
            .smartCurrentLimit(ElevatorConstants.kSmartCurrent)
            .idleMode(ElevatorConstants.kIdleMode);
        leaderConfig.encoder
        /*[TODO] constantstaki primary encoder yerine bizim encoder i koyma isi */
            .positionConversionFactor(ElevatorConstants.kPositionConversationFactor)
            .velocityConversionFactor(ElevatorConstants.kVelocityConversationFactor);
        leaderConfig.closedLoop
            .feedbackSensor(ElevatorConstants.kFeedbackSensor)
            .pid(ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD)
            .iZone(ElevatorConstants.kIZone);

        followerConfig
            .inverted(ElevatorConstants.kLeaderMotorInvert)
            .smartCurrentLimit(ElevatorConstants.kSmartCurrent)
            .idleMode(ElevatorConstants.kIdleMode);
        followerConfig.encoder
        /*[TODO] constantstaki primary encoder yerine bizim encoder i koyma isi */
            .positionConversionFactor(ElevatorConstants.kPositionConversationFactor)
            .velocityConversionFactor(ElevatorConstants.kVelocityConversationFactor);
        followerConfig.closedLoop
            .feedbackSensor(ElevatorConstants.kFeedbackSensor)
            .pid(ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD)
            .iZone(ElevatorConstants.kIZone);

        //[TODO] look in depth
        leaderSpark.configure(leaderConfig, null, null);
        followerSpark.configure(followerConfig, null, null);
    }

    /* 
    @Override
    public void close() {
        encoder.close();
        leaderSpark.close();
        mech2d.close();
    }
    */
}
