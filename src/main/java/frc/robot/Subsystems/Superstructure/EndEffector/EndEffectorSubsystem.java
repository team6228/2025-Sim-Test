package frc.robot.Subsystems.Superstructure.EndEffector;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.PWMSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class EndEffectorSubsystem extends SubsystemBase{
    private final PWMSparkMax intakeSpark = new PWMSparkMax(EndEffectorConstants.kIntakeSparkMaxPort);
    private final PWMSparkMax rotationSpark = new PWMSparkMax(EndEffectorConstants.kRotationSparkMaxPort);
    
    private final Encoder encoder = new Encoder(
        EndEffectorConstants.kEncoderChannels[0], 
        EndEffectorConstants.kEncoderChannels[1],
        EndEffectorConstants.kEncoderReversed);

    private final ProfiledPIDController controller =
      new ProfiledPIDController(
          EndEffectorConstants.kP,
          EndEffectorConstants.kI,
          EndEffectorConstants.kD,
          /* 
          //Maybe you could use just normal values
          new TrapezoidProfile.Constraints(2,5));
          */
           
          new TrapezoidProfile.Constraints(EndEffectorConstants.kMaxVelocity, 
            EndEffectorConstants.kMaxAcceleration));
        
    ArmFeedforward feedforwardController =
      new ArmFeedforward(
          EndEffectorConstants.kS,
          EndEffectorConstants.kG,
          EndEffectorConstants.kV,
          EndEffectorConstants.kA);

    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
        EndEffectorConstants.kArmGearBox,
        EndEffectorConstants.kArmReduction,
        SingleJointedArmSim.estimateMOI(EndEffectorConstants.kArmLength, EndEffectorConstants.kArmMass),
        EndEffectorConstants.kArmLength,
        EndEffectorConstants.kMinAngleRads,
        EndEffectorConstants.kMaxAngleRads,
        true,
        0,
        EndEffectorConstants.kDistancePerPulse,
        0.001);

    private final EncoderSim encoderSim = new EncoderSim(encoder);
    private final PWMSim intakeSim = new PWMSim(intakeSpark);
    private final PWMSim rotationSim = new PWMSim(rotationSpark);

    //Dumb shit
    double encoderDistance = encoder.getDistance();
    double encoderSimDistance = encoderSim.getDistance();
    double controllerValue;

    public EndEffectorSubsystem() {
        encoder.setDistancePerPulse(EndEffectorConstants.kDistancePerPulse);
    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber("Encoder distance",encoderDistance);
        SmartDashboard.putNumber("Arm angle radians", armSim.getAngleRads());
        SmartDashboard.putNumber("Encoder sim distance",encoderSimDistance);
        SmartDashboard.putNumber("PID output",controllerValue);
    }

    @Override
    public void simulationPeriodic(){
        armSim.setInput(rotationSim.getSpeed() * RobotController.getBatteryVoltage());

        armSim.update(.02);

        encoderSim.setDistance(armSim.getAngleRads());

        RoboRioSim.setVInVoltage(
            BatterySim.calculateDefaultBatteryLoadedVoltage(armSim.getCurrentDrawAmps()));

        //Dumb shit
        encoderDistance = encoder.getDistance();
        encoderSimDistance = encoderSim.getDistance();
    }

    public Command reachSetpointCmd(double setpoint){
        return this.run(() -> this.reachSetpoint(setpoint));
    }

    public Command holdAngleCmd(){
        return this.run(() -> holdAngle());
    }

    public Command stopArmCmd(){
        return this.run(() -> stopArm());
    }

    public Command testSpeedCmd(double var){
        return this.run(() -> testSpeed(var));
    }

    public void reachSetpoint(double setpoint){
        controller.setGoal(setpoint);
        double pidOutput = controller.calculate(encoder.getDistance());
        //Dumb shit
        controllerValue = pidOutput;
        SmartDashboard.putNumber("Setpoint",setpoint);
         
        double ffOutput = feedforwardController.calculate(encoder.getDistance(),controller.getSetpoint().velocity);

        //rotationSpark.setVoltage(pidOutput);
        rotationSpark.setVoltage(pidOutput + ffOutput);
    }

    public void holdAngle(){
        reachSetpoint(encoder.getDistance());
    }

    public void stopArm(){
        rotationSpark.stopMotor();
    }

    public void testSpeed(double var){
        rotationSpark.set(var);
    }

    public double setArmSimAngle(){
        return Units.radiansToDegrees(armSim.getAngleRads());
    }

    public SingleJointedArmSim getSim(){
        return armSim;
    }
}
