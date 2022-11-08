// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;

public final class Constants {

  // *****************************************
  // ************* DRIVE TRAIN ***************
  // *****************************************
  public static final class DriveConstants {

    public static final int LEFT_MOTOR = 0;
    public static final int RIGHT_MOTOR = 1;

    public static final int LEFT_A = 4;
    public static final int LEFT_B = 5;

    public static final int RIGHT_A = 6;
    public static final int RIGHT_B = 7;

    public static final double kCountsPerRevolution = 1440.0;
    public static final double kWheelDiameterMeter = 0.07; // 70 mm

    public static final double DistancePerPulse =
        (Math.PI * kWheelDiameterMeter) / kCountsPerRevolution;

    /////////////// SYSID VALUES ///////////////

    public static final double ksVolts = 0.929;
    public static final double kvVoltSecondsPerMeter = 6.33;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0389;

    public static final double kPDriveVel = 0.085;

    public static final double kTrackwidthMeters = 0.142072613;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
  }

  /////////////// AUTO STUFF ///////////////
  public static final class AutoConstants {

    // Reasonable baseline values for a RAMSETE follower in units of meters and
    // seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;

    public static final double kMaxSpeedMetersPerSecond = 0.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.8;
  }
  // *****************************************
  // ************** JOYSTICKS ****************
  // *****************************************

  public static final int DRIVERCONTROLLER = 3;

  // *****************************************
  // ************** OTHER IDC ****************
  // *****************************************

  public static final Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();
}
