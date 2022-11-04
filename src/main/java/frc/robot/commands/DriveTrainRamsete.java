// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utils.TrajectoryInjectory;

public class DriveTrainRamsete extends RamseteCommand {

  protected Boolean resetPosition;
  protected DriveTrain driveTrain;
  protected Trajectory trajectory;

  public DriveTrainRamsete(DriveTrain driveTrain, Trajectory trajectory) {
    super(
        trajectory,
        driveTrain::getPose,
        new RamseteController(Constants.kRamseteB, Constants.kRamseteZeta),
        new SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter),
        Constants.kDriveKinematics,
        driveTrain::getWheelSpeeds,
        new PIDController(Constants.kPDriveVel, 0, 0),
        new PIDController(Constants.kPDriveVel, 0, 0),
        driveTrain::tankDriveVolts,
        driveTrain);

    this.resetPosition = true;
    this.trajectory = trajectory;
    this.driveTrain = driveTrain;
  }

  public DriveTrainRamsete(DriveTrain driveTrain, String path) {
    this(driveTrain, TrajectoryInjectory.getTrajectory(path));
  }

  public DriveTrainRamsete robotRelative() {
    this.resetPosition = true;
    return this;
  }

  public DriveTrainRamsete fieldRelative() {
    this.resetPosition = false;
    return this;
  }

  @Override
  public void initialize() {
    super.initialize();

    if (resetPosition) {

      driveTrain.resetOdometry(trajectory.getInitialPose());
    }
  }
}
