// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import frc.robot.Commands.ArcadeDriveCmd;

import frc.robot.Constants.OperatorConstants;

import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Superstructure.SuperstructureSubsystem;

public class RobotContainer {
  private final DriveSubsystem mDrive = new DriveSubsystem();
  private final SuperstructureSubsystem mSuperstructure = new SuperstructureSubsystem();

  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.kJoystickPort);

  public RobotContainer() {
    configureBindings();

    mDrive.setDefaultCommand(new ArcadeDriveCmd(mDrive,() -> joystick.getRawAxis(1),() ->joystick.getRawAxis(0)));
  }

  private void configureBindings() {
    //[TODO] get button from constants
    joystick.button(1).toggleOnTrue(mSuperstructure.reachGoalCmd(1.0));
    joystick.button(2).toggleOnTrue(mSuperstructure.reachGoalCmd(0.5));


    joystick.button(3).toggleOnTrue(mSuperstructure.reachSetpointCmd(Units.degreesToRadians(45)));
    joystick.button(4).toggleOnTrue(mSuperstructure.reachSetpointCmd(Units.degreesToRadians(225)));
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
