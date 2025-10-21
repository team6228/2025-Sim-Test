package frc.robot.Subsystems.Drive;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    private final VictorSP leftMotor = new VictorSP(DriveConstants.kLeftMotorChannel); 
    private final VictorSP rightMotor = new VictorSP(DriveConstants.kRightMotorChannel);

    private final Encoder rightEncoder = new Encoder(
        DriveConstants.kRightEncoderChannels[0],
        DriveConstants.kRightEncoderChannels[1],
        DriveConstants.kLeftEncoderReversed);

    private final Encoder leftEncoder = new Encoder(
        DriveConstants.kLeftEncoderChannels[0],
        DriveConstants.kLeftEncoderChannels[1],
        DriveConstants.kLeftEncoderReversed);

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();  

    private final DifferentialDriveOdometry driveOdometry;

    private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor::set, rightMotor::set);

    private final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
    private final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);

    private final ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

    private final DifferentialDrivetrainSim robotDriveSim;

    private final Field2d fieldSim = new Field2d();

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("MyPose", Pose2d.struct).publish();

    private Pose2d robotPose;

    public DriveSubsystem() {
        SendableRegistry.addChild(robotDrive,rightMotor);
        SendableRegistry.addChild(robotDrive,leftMotor);

        rightMotor.setInverted(DriveConstants.kRightMotorReversed);

        rightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
        leftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);

        resetEncoders();

        driveOdometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()),
            leftEncoder.getRate(),
            rightEncoder.getRate());

        robotDriveSim = new DifferentialDrivetrainSim(
            DriveConstants.kDriveGearbox,
            DriveConstants.kDriveGearing,
            DriveConstants.kMoiOfRobot,
            DriveConstants.kMassOfRobot,
            DriveConstants.kWheelDiameterMeters,
            DriveConstants.kTrackWidthMeters,
            VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

        robotPose = driveOdometry.getPoseMeters();

        SmartDashboard.putData("field",fieldSim);
    }

    @Override
    public void periodic(){
        driveOdometry.update(
            Rotation2d.fromDegrees(getHeading()),
            leftEncoder.getDistance(),
            rightEncoder.getDistance());

        fieldSim.setRobotPose(getPose());
    }

    @Override
    public void simulationPeriodic(){
        robotDriveSim.setInputs(leftMotor.get() * RobotController.getBatteryVoltage(),
            rightMotor.get() * RobotController.getBatteryVoltage());
        robotDriveSim.update(0.020);

        leftEncoderSim.setDistance(robotDriveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(robotDriveSim.getLeftVelocityMetersPerSecond());

        rightEncoderSim.setDistance(robotDriveSim.getRightPositionMeters());
        rightEncoderSim.setRate(robotDriveSim.getRightVelocityMetersPerSecond());

        gyroSim.setAngle(robotDriveSim.getHeading().getDegrees());

        robotPose = driveOdometry.getPoseMeters();
        publisher.set(robotPose);
    }

    public Pose2d getPose(){
        return driveOdometry.getPoseMeters();
    }

    public void resetEncoders(){
        rightEncoder.reset();
        leftEncoder.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void arcadeDrive(double fwd,Double rot){
        robotDrive.arcadeDrive(fwd * DriveConstants.kFwdCap * DriveConstants.kFwdRot
        ,rot * DriveConstants.kRotCap * DriveConstants.kRotRot);
    }
}