// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {

  public static double kMaxSpeed = 3.0; // 3 meters per second [AVG MAX 3.5]
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

  private final Translation2d frontLeftLocation = new Translation2d(0.335, 0.335);
  private final Translation2d frontRightLocation = new Translation2d(0.335, -0.335);
  private final Translation2d backLeftLocation = new Translation2d(-0.335, 0.335);
  private final Translation2d backRightLocation = new Translation2d(-0.335, -0.335);

  public SwerveModule frontLeft = new SwerveModule(10, 11, 0, 0.62);
  public SwerveModule frontRight = new SwerveModule(12, 13, 1, 0.23);
  public SwerveModule backLeft = new SwerveModule(14, 15, 2, 0.95);
  public SwerveModule backRight = new SwerveModule(16, 17, 3, 0.87);

  public AHRS navx = new AHRS(SPI.Port.kMXP);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  public final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          navx.getRotation2d(),
          new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
          });

  public Drivetrain() {
    // navx.reset(); // Calibrates the gyro then resets the yaw, pitch, and roll to zero.
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, navx.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    backLeft.setDesiredState(swerveModuleStates[2]);
    backRight.setDesiredState(swerveModuleStates[3]);

    SmartDashboard.putNumber("LeftFront Dis.", frontLeft.m_turningEncoder.getDistance());
    SmartDashboard.putNumber("RightFront Dis.", frontRight.m_turningEncoder.getDistance());
    SmartDashboard.putNumber("LeftBack Dis.", backLeft.m_turningEncoder.getDistance());
    SmartDashboard.putNumber("RightBack Dis.", backRight.m_turningEncoder.getDistance());
    
    SmartDashboard.putNumber("LeftFront State", swerveModuleStates[0].angle.getRadians());
    SmartDashboard.putNumber("RightFront State", swerveModuleStates[1].angle.getRadians());
    SmartDashboard.putNumber("LeftBack State", swerveModuleStates[2].angle.getRadians());
    SmartDashboard.putNumber("RightBack State", swerveModuleStates[3].angle.getRadians());
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        navx.getRotation2d(),
        new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
        });
  }

  // Finds the average of all drive encoders.
  public double getDistanceAVG() {
    return (frontLeft.getDistance() + frontRight.getDistance() + backLeft.getDistance() + backRight.getDistance()) / 4;
  }

  // resets and calibrations
  public void resetEncoder() {
    frontLeft.m_driveEncoder.setPosition(0);
    frontRight.m_driveEncoder.setPosition(0);
    backLeft.m_driveEncoder.setPosition(0);
    backRight.m_driveEncoder.setPosition(0);
  }

  public void resetGyro() {
    navx.reset();
  }

  // Reads gyro (between 0-360).
  public double robotBearing() {
    return navx.getAngle() % 360;
  }
  
  public double robotPitch() {
    return navx.getPitch() % 360;
  }

  public double robotRoll() {
    return navx.getRoll() % 360;
  }
}