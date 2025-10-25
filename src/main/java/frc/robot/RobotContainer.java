// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import frc.robot.Commands.ArcadeDriveCmd;

import frc.robot.Constants.OperatorConstants;

import frc.robot.Subsystems.Drive.DriveSubsystem;
import frc.robot.Subsystems.Superstructure.Elevator.ElevatorSubsystem;

public class RobotContainer {
  private final DriveSubsystem mDriveSubsystem = new DriveSubsystem();
  private final ElevatorSubsystem mElevatorSubsystem = new ElevatorSubsystem();

  private final CommandJoystick joystick = new CommandJoystick(OperatorConstants.kJoystickPort);

  public RobotContainer() {
    configureBindings();

    mDriveSubsystem.setDefaultCommand(new ArcadeDriveCmd(mDriveSubsystem,() -> joystick.getRawAxis(1),() ->joystick.getRawAxis(0)));
  }

  private void configureBindings() {
    //[TODO] get button from constants
    joystick.button(1).toggleOnTrue(mElevatorSubsystem.reachGoalCmd(1.25));
    joystick.button(2).toggleOnTrue(mElevatorSubsystem.reachGoalCmd(1.0));
    joystick.button(3).toggleOnTrue(mElevatorSubsystem.reachGoalCmd(0.5));
    joystick.button(4).toggleOnTrue(mElevatorSubsystem.reachGoalCmd(0.0));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
