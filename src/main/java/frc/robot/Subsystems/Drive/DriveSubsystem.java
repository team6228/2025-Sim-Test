package frc.robot.Subsystems.Drive;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import frc.robot.Subsystems.Drive.DriveConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase{
    private final VictorSP leftMotor = new VictorSP(DriveConstants.kLeftMotorChannel); 
    private final VictorSP rightMotor = new VictorSP(DriveConstants.kRightMotorChannel);
    

    private final DifferentialDrive robotDrive = new DifferentialDrive(leftMotor, rightMotor);

    public DriveSubsystem() {}

    @Override
    public void periodic(){
    }

    @Override
    public void simulationPeriodic(){

    }

    public void arcadeDrive(double fwd,Double rot){
        robotDrive.arcadeDrive(fwd * DriveConstants.kFwdCap, rot * DriveConstants.kRotCap);
    }
}