package frc.robot.Subsystems.Drive;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    private final VictorSP leftFrontMotor = new VictorSP(DriveConstants.kLeftFrontMotorChannel); 
    private final VictorSP rightFrontMotor = new VictorSP(DriveConstants.kRightFrontMotorChannel);
    private final VictorSP leftRearMotor = new VictorSP(DriveConstants.kLeftRearMotorChannel);
    private final VictorSP rightRearMotor = new VictorSP(DriveConstants.kRightRearMotorChannel);

    //[TODO] Edit
    private final Translation2d frontLeftLocation = new Translation2d(0.381, 0.381);
    private final Translation2d frontRightLocation = new Translation2d(0.381, -0.381);
    private final Translation2d backLeftLocation = new Translation2d(-0.381, 0.381);
    private final Translation2d backRightLocation = new Translation2d(-0.381, -0.381);

    private final Encoder leftFrontEncoder = new Encoder(
        DriveConstants.kLeftFrontEncoderChannels[0],
        DriveConstants.kLeftFrontEncoderChannels[1],
        DriveConstants.kLeftFrontEncoderReversed);

    private final Encoder rightFrontEncoder = new Encoder(
        DriveConstants.kRightFrontEncoderChannels[0],
        DriveConstants.kRightFrontEncoderChannels[1],
        DriveConstants.kRightFrontEncoderReversed);

    private final Encoder leftRearEncoder = new Encoder(
        DriveConstants.kLeftRearEncoderChannels[0],
        DriveConstants.kLeftRearEncoderChannels[1],
        DriveConstants.kLeftRearEncoderReversed);
    
    private final Encoder rightRearEncoder = new Encoder(
        DriveConstants.kRightRearEncoderChannels[0],
        DriveConstants.kRightRearEncoderChannels[1],
        DriveConstants.kRightRearEncoderReversed);

    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();  

    private final MecanumDriveOdometry driveOdometry;
    private final MecanumDriveKinematics robotKinematics = new MecanumDriveKinematics(frontRightLocation, frontLeftLocation, backRightLocation, backLeftLocation);
    private final MecanumDrive robotDrive = new MecanumDrive(leftFrontMotor::set, leftRearMotor::set, rightFrontMotor::set, rightRearMotor::set);

    private final EncoderSim leftFrontEncoderSim = new EncoderSim(leftFrontEncoder);
    private final EncoderSim rightFrontEncoderSim = new EncoderSim(rightFrontEncoder);
    private final EncoderSim leftRearEncoderSim = new EncoderSim(leftRearEncoder);
    private final EncoderSim rightRearEncoderSim = new EncoderSim(rightRearEncoder);

    private final ADXRS450_GyroSim gyroSim = new ADXRS450_GyroSim(gyro);

    //Robot drive sim

    private final Field2d fieldSim = new Field2d();

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
        .getStructTopic("MyPose", Pose2d.struct).publish();

    private Pose2d robotPose;

    public DriveSubsystem() {
        SendableRegistry.addChild(robotDrive,leftFrontMotor);
        SendableRegistry.addChild(robotDrive,rightFrontMotor);
        SendableRegistry.addChild(robotDrive,leftRearMotor);
        SendableRegistry.addChild(robotDrive,rightRearMotor);

        leftFrontMotor.setInverted(DriveConstants.kLeftFrontMotorReversed);
        rightFrontMotor.setInverted(DriveConstants.kRightFrontMotorReversed);
        leftRearMotor.setInverted(DriveConstants.kLeftRearMotorReversed);
        rightRearMotor.setInverted(DriveConstants.kRightRearMotorReversed);

        leftFrontEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
        rightFrontEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
        leftRearEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);
        rightRearEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulse);

        //Reset stuff
        resetEncoders();
        gyro.reset();

        //Kinematics,Odometry,Drive
        driveOdometry = new MecanumDriveOdometry(robotKinematics, Rotation2d.fromDegrees(getHeading()), 
            new MecanumDriveWheelPositions(
                leftFrontEncoder.getDistance(),rightFrontEncoder.getDistance(),
                leftRearEncoder.getDistance(),rightRearEncoder.getDistance()));

        robotPose = driveOdometry.getPoseMeters();

        SmartDashboard.putData("field",fieldSim);
    }

    @Override
    public void periodic(){
        var wheelPositions = new MecanumDriveWheelPositions(leftFrontEncoder.getDistance(),rightFrontEncoder.getDistance(),
            leftRearEncoder.getDistance(),rightRearEncoder.getDistance());
        driveOdometry.update(Rotation2d.fromDegrees(getHeading()),wheelPositions);

        fieldSim.setRobotPose(getPose());
    }

    @Override
    public void simulationPeriodic(){
        //[TODO] Set encoder,gyro sims

        publisher.set(getPose());
    }

    public Pose2d getPose(){
        return driveOdometry.getPoseMeters();
    }

    public void resetEncoders(){
        leftFrontEncoder.reset();
        rightFrontEncoder.reset();
        leftRearEncoder.reset();
        rightRearEncoder.reset();
    }

    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public void cartesianDrive(Double speedX,Double speedY,Double rotationZ){
        robotDrive.driveCartesian(speedX * DriveConstants.kWheelSpeedCap * -1.0,
            speedY * DriveConstants.kWheelSpeedCap * -1.0,rotationZ * DriveConstants.kWheelSpeedCap * -1.0);
    }

    //[TODO] learn polar drive
    public void polarDrive(){
        robotDrive.drivePolar(0,Rotation2d.fromDegrees(0),0);
    }
}