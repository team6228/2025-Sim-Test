package frc.robot.Subsystems.Elevator;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends SubsystemBase{
    private final PWMSparkMax followerSpark = new PWMSparkMax(ElevatorConstants.kFollowerSparkMaxPort);
    private final PWMSparkMax leaderSpark = new PWMSparkMax(ElevatorConstants.kLeaderSparkMaxPort);

    private final Encoder encoder = new Encoder(
        ElevatorConstants.kEncoderChannels[0],
        ElevatorConstants.kEncoderChannels[1],
        ElevatorConstants.kEncoderReversed);

    //[TODO] Feedback controllers
    //[TODO] May use pwm sim

    private final ProfiledPIDController controller =
      new ProfiledPIDController(
          ElevatorConstants.kP,
          ElevatorConstants.kI,
          ElevatorConstants.kD,
          new TrapezoidProfile.Constraints(1.0, 1.0));
  ElevatorFeedforward feedforwardController =
      new ElevatorFeedforward(
          ElevatorConstants.kS,
          ElevatorConstants.kG,
          ElevatorConstants.kV,
          ElevatorConstants.kA);

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
    0);

    private final EncoderSim encoderSim = new EncoderSim(encoder);
    private final PWMSim followerSim = new PWMSim(followerSpark);
    private final PWMSim leaderSim = new PWMSim(leaderSpark);

    //[TODO look into them values]
    private final Mechanism2d mech2d = new Mechanism2d(2,2);
    private final MechanismRoot2d mech2droot = mech2d.getRoot("Elevator root", 1, 0);
    private final MechanismLigament2d elevatorMech2d = mech2droot.append(
        new MechanismLigament2d("Elevator", elevatorSim.getPositionMeters(), 90)
    );

    public ElevatorSubsystem() {
        encoder.reset();
        encoder.setDistancePerPulse(ElevatorConstants.kDistancePerPulse);

        leaderSpark.addFollower(followerSpark);

        SmartDashboard.putData("Elevator Sim",mech2d);
    }

    @Override
    public void periodic(){
        elevatorMech2d.setLength(encoder.getDistance());
    }

    @Override
    public void simulationPeriodic(){
        //[TODO] may use sim motor
        //[TODO] may use something other than getVelocity
        elevatorSim.setInput(leaderSim.getSpeed() * RobotController.getBatteryVoltage() 
        + followerSim.getSpeed() * RobotController.getBatteryVoltage());

        elevatorSim.update(0.020);

        encoderSim.setDistance(elevatorSim.getPositionMeters());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(elevatorSim.getCurrentDrawAmps()));
    }

    public Command reachGoalCmd(double goal){
        return this.run(() -> reachGoal(goal));
    }

    public void reachGoal(double goal){
        //[TODO] implement controller

        controller.setGoal(goal);

        // With the setpoint value we run PID control like normal
        double pidOutput = controller.calculate(encoder.getDistance());
        double feedforwardOutput = feedforwardController.calculate(
            controller.getSetpoint().velocity);
        leaderSpark.setVoltage(pidOutput + feedforwardOutput);
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
