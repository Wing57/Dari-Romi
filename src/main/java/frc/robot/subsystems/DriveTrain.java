// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {

  private final Spark m_leftMotor;
  private final Spark m_rightMotor;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final DifferentialDrive drive;

  private final RomiGyro m_gyro;

  /** Creates a new Drivetrain. */
  public DriveTrain() {
    /*** MOTORS ***/
    m_leftMotor = new Spark(Constants.LEFT_MOTOR);
    m_rightMotor = new Spark(Constants.RIGHT_MOTOR);

    m_rightMotor.setInverted(true);

    // Create DifferentialDrive object
    drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    // Initialize gyroscope
    m_gyro = new RomiGyro();

    /*** ENCODERS ***/
    m_leftEncoder = new Encoder(Constants.LEFT_A, Constants.LEFT_B);
    m_rightEncoder = new Encoder(Constants.RIGHT_A, Constants.RIGHT_B);

    m_leftEncoder.setDistancePerPulse(Constants.DistancePerPulse);
    m_rightEncoder.setDistancePerPulse(Constants.DistancePerPulse);
    resetEncoders();
  }

  public void drive(double left, double right) {
    drive.tankDrive(left, right);
  }

  // *****************************************
  // ************** Encoders *****************
  // *****************************************

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /*** Distance ***/
  public double getLeftDistance() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  /*** Velocity ***/
  public double getLeftVelocity() {
    return m_leftEncoder.getRate();
  }

  public double getRightVelocity() {
    return m_rightEncoder.getRate();
  }

  public double getVelocity() {
    return (getLeftVelocity() + getRightVelocity()) / 2.0;
  }
  
  // *****************************************
  // ************* Robot Angle ***************
  // *****************************************
  
  // Should be the same as navX.getangle()
  public double getGyroAngle() {
    return m_gyro.getAngleZ();
  }

  

  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
