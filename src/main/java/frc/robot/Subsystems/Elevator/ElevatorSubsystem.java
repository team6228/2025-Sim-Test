package frc.robot.Subsystems.Elevator;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

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

    private final SparkMaxSim leaderSparkSim = new SparkMaxSim(leaderSpark, DCMotor.getNEO(1));
    private final SparkMaxSim followerSparkSim = new SparkMaxSim(followerSpark, DCMotor.getNEO(1));

    private final EncoderSim encoderSim = new EncoderSim(encoder);

    private final Mechanism2d mech2d = new Mechanism2d(20,50);
    private final MechanismRoot2d mech2droot = mech2d.getRoot("Elevator root", 10, 0);
    private final MechanismLigament2d elevatorMech2d = mech2droot.append(
        new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90)
    );

    public ElevatorSubsystem() {
        encoder.reset();
        encoder.setDistancePerPulse(ElevatorConstants.kDistancePerPulse);

        followerSpark.isFollower();

        setSparkMaxConfig();

        SmartDashboard.putData("Elevator Sim",mech2d);
    }

    @Override
    public void periodic(){

    }

    @Override
    public void simulationPeriodic(){
        //[TODO] may use sim motor
        //[TODO] may use something other than getVelocity
        System.out.println(leaderSparkSim.getAppliedOutput());
        elevatorSim.setInput(leaderSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage() 
            + followerSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());

        elevatorSim.update(0.020);

        encoderSim.setDistance(elevatorSim.getPositionMeters());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

        elevatorMech2d.setLength(encoder.getDistance());
    }

    public Command testElevatorCmd(){
        return this.startEnd(() -> this.testElevator(5),() -> this.testElevator(0));
    }

    public void reachGoal(double goal){
        //[TODO] implement controller

        double pidOutput = 0.0;
        double feedForward = 0.0;

        leaderSpark.setVoltage(pidOutput + feedForward);
    }

    public void testElevator(double voltage){
        //[TODO] delete and look into logging
        leaderSpark.setVoltage(voltage);
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
            //[TODO] feedbackSensor verisini constantstan ek
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD)
            .iZone(ElevatorConstants.kIZone);

        followerConfig
            .follow(ElevatorConstants.kLeaderSparkMaxCanId)
            .inverted(ElevatorConstants.kFollowerMotorInvert)
            .smartCurrentLimit(ElevatorConstants.kSmartCurrent)
            .idleMode(ElevatorConstants.kIdleMode);
        followerConfig.encoder
        /*[TODO] constantstaki primary encoder yerine bizim encoder i koyma isi */
            .positionConversionFactor(ElevatorConstants.kPositionConversationFactor)
            .velocityConversionFactor(ElevatorConstants.kVelocityConversationFactor);
        followerConfig.closedLoop
            //[TODO] feedbackSensor verisini constantstan ek
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
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
