// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  // *****************************************
  // ************** JOYSTICKS ****************
  // *****************************************

  public static final int DRIVERCONTROLLER = 3;
}
