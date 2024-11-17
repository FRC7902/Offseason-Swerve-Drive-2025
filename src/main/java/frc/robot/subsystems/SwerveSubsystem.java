// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;

public class SwerveSubsystem extends SubsystemBase {

  private SwerveDrive m_swerveDrive;

  SwerveModuleState[] states;

  private final StructArrayPublisher<SwerveModuleState> m_moduleStatePublisher;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

    try {
      File directory = new File(Filesystem.getDeployDirectory(), "swerve");
      m_swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    m_moduleStatePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

  }

  public Pose2d getPose() {
    return m_swerveDrive.getPose();
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    m_swerveDrive.resetOdometry(initialHolonomicPose);
  }

  public ChassisSpeeds getRobotVelocity() {
    return m_swerveDrive.getRobotVelocity();
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    m_swerveDrive.setChassisSpeeds(chassisSpeeds);
  }

  public void initializePathPlanner() {
      AutoBuilder.configureHolonomic(
        this::getPose, // Provides robot pose (combination of translation and rotation)
        this::resetOdometry, // Resets odometry (runs when auto has a starting pose)
        this::getRobotVelocity, // Provides chassis velocity
        this::setChassisSpeeds, // Sets robot speed 
        new HolonomicPathFollowerConfig(
                                        Constants.AutonConstants.TRANSLATION_PID,
                                        Constants.AutonConstants.ANGLE_PID,
                                        Constants.AutonConstants.MAX_MODULE_SPEED,
                                        m_swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                                        new ReplanningConfig()),
                                        () -> {
                                          var alliance = DriverStation.getAlliance();
                                          return (alliance.isPresent()) ? (alliance.get() == DriverStation.Alliance.Red) : (false);},
                                          this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    states = m_swerveDrive.getStates();
    SmartDashboard.putNumber("FL Angle (deg)", states[0].angle.getDegrees());
    SmartDashboard.putNumber("FR Angle (deg)", states[1].angle.getDegrees());
    SmartDashboard.putNumber("BL Angle (deg)", states[2].angle.getDegrees());
    SmartDashboard.putNumber("BR Angle (deg)", states[3].angle.getDegrees());

    SmartDashboard.putNumber("FL Speed (m/s)", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FR Speed (m/s)", states[1].speedMetersPerSecond);
    SmartDashboard.putNumber("BL Speed (m/s)", states[2].speedMetersPerSecond);
    SmartDashboard.putNumber("BR Speed (m/s)", states[3].speedMetersPerSecond);

    SwerveModuleState[] states = m_swerveDrive.getStates();
    m_moduleStatePublisher.set(states);
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth controll out
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth controll out
      // Make the robot move
      driveFieldOriented(m_swerveDrive.swerveController.getTargetSpeeds(xInput * 0.1, yInput * 0.1,
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          m_swerveDrive.getYaw().getRadians(),
          m_swerveDrive.getMaximumVelocity()));
    });
  }

  /**
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */
  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      m_swerveDrive.drive(
          new Translation2d(
              translationX.getAsDouble() * m_swerveDrive.getMaximumVelocity(),
              translationY.getAsDouble() * m_swerveDrive
                  .getMaximumVelocity()),
          angularRotationX.getAsDouble() * m_swerveDrive.getMaximumAngularVelocity(),
          true,
          false);
    });
  }

  /**
   * Drive the robot given a chassis field oriented velocity.
   *
   * @param velocity Velocity according to the field.
   */
  public void driveFieldOriented(ChassisSpeeds velocity) {
    m_swerveDrive.driveFieldOriented(velocity);
  }

  /**
   * Gets the current field-relative pose of the robot according to odometry.
   * 
   * @return The current robot pose.
   */

}
