package frc.robot.Subsystems.Elevator;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkMaxAlternateEncoderSim;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkMaxAlternateEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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

    private final SparkClosedLoopController controller = leaderSpark.getClosedLoopController();

    private final Encoder encoder = new Encoder(
        ElevatorConstants.kEncoderChannels[0],
        ElevatorConstants.kEncoderChannels[1],
        ElevatorConstants.kEncoderReversed);

    private final RelativeEncoder sparkEncoder = leaderSpark.getEncoder(); 

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
    0.001,
    0.0);

    private final SparkMaxSim leaderSparkSim = new SparkMaxSim(leaderSpark, DCMotor.getNEO(2));

    private final EncoderSim encoderSim = new EncoderSim(encoder);

    private SparkRelativeEncoderSim sparkEncoderSim = new SparkRelativeEncoderSim(leaderSpark);

    private final Mechanism2d mech2d = new Mechanism2d(2,2);
    private final MechanismRoot2d mech2droot = mech2d.getRoot("Elevator root", 1, 0);
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
        sparkEncoderSim.setPosition(elevatorSim.getPositionMeters());

        //[TODO] may use sim motor
        //[TODO] may use something other than getVelocity
        elevatorSim.setInput(leaderSparkSim.getAppliedOutput() * RobotController.getBatteryVoltage());

        elevatorSim.update(0.020);

        encoderSim.setDistance(elevatorSim.getPositionMeters());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));

        elevatorMech2d.setLength(encoder.getDistance());
    }

    public Command reachGoalCmd(double goal){
        return this.run(() -> reachGoal(goal));
    }

    public void reachGoal(double goal){
        //[TODO] implement controller
        controller.setReference(goal, ControlType.kPosition);
    }

    public void setSparkMaxConfig(){
        leaderConfig
            .inverted(ElevatorConstants.kLeaderMotorInvert)
            .smartCurrentLimit(ElevatorConstants.kSmartCurrent)
            .idleMode(ElevatorConstants.kIdleMode);
        leaderConfig.encoder
        /*[TODO] constantstaki primary encoder yerine bizim encoder i koyma isi */
            //.countsPerRevolution((int) ElevatorConstants.kEncoderCPR)
            //.inverted(ElevatorConstants.kEncoderReversed)
            .positionConversionFactor(ElevatorConstants.kPositionConversationFactor)
            .velocityConversionFactor(ElevatorConstants.kVelocityConversationFactor);
        leaderConfig.closedLoop
            //[TODO] feedbackSensor verisini constantstan ek
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.kP,ElevatorConstants.kI,ElevatorConstants.kD)
            //[TODO] add this to constants
            .outputRange(0, 1.25)
            .iZone(ElevatorConstants.kIZone);
        //===================================
        followerConfig
            .follow(ElevatorConstants.kLeaderSparkMaxCanId);

        //[TODO] look in depth
        leaderSpark.configure(leaderConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        followerSpark.configure(followerConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
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
