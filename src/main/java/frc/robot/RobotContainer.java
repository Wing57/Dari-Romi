// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.rambots4571.rampage.joystick.Controller;
import com.rambots4571.rampage.joystick.Gamepad;

import frc.robot.commands.TankDriveCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

public class RobotContainer {

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Joysticks
  public static final Controller<Gamepad.Button, Gamepad.Axis> driveController =
  Gamepad.make(Constants.DRIVERCONTROLLER);

  // Subsystems
  private final DriveTrain driveTrain;
  private final OnBoardIO m_onboardIO;

  // Commands
  private final TankDriveCommand tankDriveCommand;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    driveTrain = new DriveTrain();
    m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

    tankDriveCommand = new TankDriveCommand(driveTrain);

    driveTrain.setDefaultCommand(tankDriveCommand);

    configureButtonBindings();
  }

  private void configureButtonBindings() {

    // Example of how to use the onboard IO
    Button onboardButtonA = new Button(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whenActive(new PrintCommand("Button A Pressed"))
        .whenInactive(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    SmartDashboard.putData(m_chooser);
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
