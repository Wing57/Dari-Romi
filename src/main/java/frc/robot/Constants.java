// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.Filesystem;

public final class Constants {

  // *****************************************
  // ************* DRIVE TRAIN ***************
  // *****************************************

  public static final int LEFT_MOTOR = 0;
  public static final int RIGHT_MOTOR = 1;

  public static final int LEFT_A = 4;
  public static final int LEFT_B = 5;

  public static final int RIGHT_A = 6;
  public static final int RIGHT_B = 7;

  public static final double DistancePerPulse = (Math.PI * Constants.kWheelDiameterInch) / Constants.kCountsPerRevolution;

  public static final double kCountsPerRevolution = 1440.0;
  public static final double kWheelDiameterInch = 2.75591; // 70 mm

  /////////////// SYSID VALUES ///////////////

  public static final double ksVolts = 0.558;
  public static final double kvVoltSecondsPerMeter = 3.4335;
  public static final double kaVoltSecondsSquaredPerMeter = 0.171;

  public static final double kPDriveVel = 3.2137;

  //TODO: Find real values for all of these innit
  public static final double kMaxVelocityMetersPerSecond = 0.0;
  public static final double kMaxAccelerationMetersPerSecondSquared = 0.0;

  public static final double kTrackwidthMeters = 0.74551;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);
  
  /////////////// AUTO STUFF ///////////////

  // Reasonable baseline values for a RAMSETE follower in units of meters and
  // seconds
  public static final double kRamseteB = 2;
  public static final double kRamseteZeta = 0.7;
  
  // *****************************************
  // ************** JOYSTICKS ****************
  // *****************************************

  public static final int DRIVERCONTROLLER = 3;

  // *****************************************
  // ************** OTHER IDC ****************
  // *****************************************

  public static final Path DEPLOY_DIRECTORY = Filesystem.getDeployDirectory().toPath();
}
