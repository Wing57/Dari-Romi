// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.utils.RomiGyro;

public class DriveTrain extends SubsystemBase {

  private final Spark m_leftMotor;
  private final Spark m_rightMotor;

  private final Encoder m_leftEncoder;
  private final Encoder m_rightEncoder;

  private final DifferentialDrive drive;

  private final RomiGyro m_gyro;

  private final DifferentialDriveOdometry m_Odometry;

  private final Field2d field;

  /** Creates a new Drivetrain. */
  public DriveTrain() {
    /*** MOTORS ***/
    m_leftMotor = new Spark(DriveConstants.LEFT_MOTOR);
    m_rightMotor = new Spark(DriveConstants.RIGHT_MOTOR);

    m_rightMotor.setInverted(true);

    // Initialize gyroscope
    m_gyro = new RomiGyro();

    // Initialize field2d
    field = new Field2d();

    // Create DifferentialDrive object
    drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

    // Create Odometry object
    m_Odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d());

    // Put field on the spot RUHEUHEUHEHE
    SmartDashboard.putData("Field", field);

    /*** ENCODERS ***/
    m_leftEncoder = new Encoder(DriveConstants.LEFT_A, DriveConstants.LEFT_B);
    m_rightEncoder = new Encoder(DriveConstants.RIGHT_A, DriveConstants.RIGHT_B);

    m_leftEncoder.setDistancePerPulse(DriveConstants.DistancePerPulse);
    m_rightEncoder.setDistancePerPulse(DriveConstants.DistancePerPulse);
    resetEncoders();
  }

  // *****************************************
  // ************** Driving ******************
  // *****************************************

  public void drive(double left, double right) {
    drive.tankDrive(left, right);
  }

  public void stopMotors() {
    drive.stopMotor();
  }

  // ********************************************
  // ************ Odometry Functions ************
  // ********************************************

  public void updateOdometry() {
    m_Odometry.update(m_gyro.getRotation2d(), getLeftDistance(), getRightDistance());
  }

  public Pose2d getPose() {
    updateOdometry();
    return m_Odometry.getPoseMeters();
  }

  public Field2d getField() {
    return field;
  }

  // TODO: get the speed in m/s and not raw units
  // find the distance per pulse.
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  public void resetOdometry(Pose2d pose2d) {
    m_gyro.reset();
    resetEncoders();
    m_Odometry.resetPosition(pose2d, m_gyro.getRotation2d());
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

  // Returns The robots heading in degrees, from -180 to 180
  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }

  public double getTurnRate() {
    return m_gyro.getRate();
  }

  public void resetGyro() {
    m_gyro.reset();
  }

  // *****************************************
  // ************** Voltage ******************
  // *****************************************

  public void tankDriveVolts(double leftVolts, double rightVolts) {
    m_leftMotor.setVoltage(leftVolts);
    m_rightMotor.setVoltage(rightVolts);
    drive.feed();
  }

  public void setMaxOutput(double MaxOutput) {
    drive.setMaxOutput(MaxOutput);
  }

  @Override
  public void periodic() {
    updateOdometry();
    field.setRobotPose(getPose());
  }

  public void InitSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Angle", this::getGyroAngle, null);
  }
}
