// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;

class SwerveModule {
  private Talon driveMotor;
  private Talon steerMotor;

  private SwerveModuleState currentState;
  private SwerveModuleState desiredState;

  public SwerveModule(int driveMotorPort, int steeringMotorPort) {
    // Initialize motors
    driveMotor = new Talon(driveMotorPort);
    steerMotor = new Talon(steeringMotorPort);

    // Initialize current state
    currentState = new SwerveModuleState();
  }

  public SwerveModuleState getState() {
    return currentState;  // Returns speed (m/s) and angle (radians)
  }

  public void setDesiredState(SwerveModuleState newState) {
    // Optimize the desired state
    desiredState = SwerveModuleState.optimize(newState, getCurrentAngle());

    double angleError = desiredState.angle.getRadians() - getCurrentAngle().getRadians();

    currentState = new SwerveModuleState(
        desiredState.speedMetersPerSecond * Math.cos(angleError),
        desiredState.angle);
  }

  private Rotation2d getCurrentAngle() {
    return currentState.angle; // For simulation, we can just return the current angle
  }
}

public class SwerveSubsystem extends SubsystemBase {
  SwerveDrive m_swerveDrive;
  StructArrayPublisher<SwerveModuleState> publisher;

  SwerveModule frontLeftModule = new SwerveModule(0, 1);
  SwerveModule frontRightModule = new SwerveModule(2, 3);
  SwerveModule backLeftModule = new SwerveModule(4, 5);
  SwerveModule backRightModule = new SwerveModule(6, 7);

  double chassisWidth = Units.inchesToMeters(10);
  double chassisLength = Units.inchesToMeters(10);

  // Defining the locations of the wheels on the robot relative to its
  // Let forward be [+] and left be [+]
  Translation2d frontLeftLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
  Translation2d frontRightLocation = new Translation2d(chassisLength / 2, -chassisWidth / 2);
  Translation2d backLeftLocation = new Translation2d(-chassisLength / 2, chassisWidth / 2);
  Translation2d backRightLocation = new Translation2d(-chassisLength / 2, -chassisWidth / 2);

  // Define a kinematics object
  // Takes chassis speeds and returns swerve module states
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation,
      frontRightLocation,
      backLeftLocation,
      backRightLocation);

  // Get a reference to the controller
  CommandXboxController controller;

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem(CommandXboxController io) {
    controller = io;

    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, DriveConstants.ENCODER_PPR);

    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION)
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.86,
        DriveConstants.ENCODER_PPR);

    System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
    System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");

    try {
      File directory = new File(Filesystem.getDeployDirectory(), "swerve");
      // Assuming you have a method to create a swerve drive from configuration files
      m_swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    publisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();
  }

  public void setChassisSpeed(ChassisSpeeds desiredSpeed) {
    // Get the desired states of the wheels
    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desiredSpeed);

    frontLeftModule.setDesiredState(newStates[0]);
    frontRightModule.setDesiredState(newStates[1]);
    backLeftModule.setDesiredState(newStates[2]);
    backRightModule.setDesiredState(newStates[3]);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Read data from the controller
    // Get the x and y inputs from the left joystick
    ChassisSpeeds newDesiredSpeed = new ChassisSpeeds(
        controller.getLeftX(), // Move forward - axis 1
        controller.getLeftY(), // Move left - axis 0
        controller.getRightX() // Rotate left - axis 4
    );

    setChassisSpeed(newDesiredSpeed);

    // Periodically send a set of module states
    publisher.set(new SwerveModuleState[] {
        frontLeftModule.getState(),
        frontRightModule.getState(),
        backLeftModule.getState(),
        backRightModule.getState()
    });
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
    return run(() -> {
      double xInput = Math.pow(translationX.getAsDouble(), 3); // Smooth control
      double yInput = Math.pow(translationY.getAsDouble(), 3); // Smooth control

      driveFieldOriented(m_swerveDrive.swerveController.getTargetSpeeds(
          xInput, yInput, headingX.getAsDouble(), headingY.getAsDouble(), m_swerveDrive.getYaw().getRadians(),
          m_swerveDrive.getMaximumVelocity()));
    });
  }

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      m_swerveDrive.drive(
          new Translation2d(translationX.getAsDouble() * m_swerveDrive.getMaximumVelocity(),
              translationY.getAsDouble() * m_swerveDrive.getMaximumVelocity()),
          angularRotationX.getAsDouble() * m_swerveDrive.getMaximumAngularVelocity(),
          true, false);
    });
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    m_swerveDrive.driveFieldOriented(velocity);
  }
}