// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

<<<<<<< Updated upstream
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
=======
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> Stashed changes
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import swervelib.SwerveDrive;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

<<<<<<< Updated upstream
=======
class SwerveModule {
  private Talon driveMotor;
  private Talon steerMotor;
  private SwerveModuleState currentState;

  public SwerveModule(int driveMotorPort, int steeringMotorPort) {
    driveMotor = new Talon(driveMotorPort);
    steerMotor = new Talon(steeringMotorPort);
    currentState = new SwerveModuleState();
    SwerveModuleState currentState = new SwerveModuleState(1, new Rotation2d(Units.degreesToRadians(30)));
  }

public SwerveModuleState getState() {
  return currentState;
}

public void setState(SwerveModuleState newState) {
  currentState = newState;
}

}


>>>>>>> Stashed changes
public class SwerveSubsystem extends SubsystemBase {

  SwerveDrive m_swerveDrive;

<<<<<<< Updated upstream
  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
=======
  SwerveModule frontLeftModule = new SwerveModule(0,1);
  SwerveModule frontRightModule = new SwerveModule(2,3);
  SwerveModule backLeftModule = new SwerveModule(4,5);
  SwerveModule backRightModule = new SwerveModule(6,7);
  
  double chassisWidth = Units.inchesToMeters(0);
  double chassisLength = Units.inchesToMeters(0);

  // Defining the locations of the wheels on the robot relative to its center
  // Let up be [+] and left be [+]
  Translation2d frontLeftLocation = new Translation2d(chassisLength / 2, chassisWidth / 2);
  Translation2d frontRightLocation = new Translation2d(chassisLength / 2, -chassisWidth / 2);
  Translation2d backLeftLocation = new Translation2d(-chassisLength / 2, chassisWidth / 2);
  Translation2d backRightLocation = new Translation2d(-chassisLength / 2, -chassisWidth / 2);


  // Define a kinematics object
  // Takes ChassisSpeeds and returns SwerveModuleStates
  SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    frontLeftLocation,
    frontRightLocation,
    backLeftLocation,
    backRightLocation
  );

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {


>>>>>>> Stashed changes
    // Angle conversion factor is 360 / (GEAR RATIO * ENCODER RESOLUTION)
    double angleConversionFactor = SwerveMath.calculateDegreesPerSteeringRotation(12.8, DriveConstants.ENCODER_PPR); // TODO: Add encoder PPR value
    // Motor conversion factor is (PI * WHEEL DIAMETER IN METERS) / (GEAR RATIO * ENCODER RESOLUTION)
    double driveConversionFactor = SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.86, DriveConstants.ENCODER_PPR); // TODO: Add encoder PPR value
    System.out.println("\t\"angle\": {\"factor\": " + angleConversionFactor + " },");
    System.out.println("\t\"drive\": {\"factor\": " + driveConversionFactor + " }");
    System.out.println("}");
  
    // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created
    // SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;

    try {
      File directory = new File(Filesystem.getDeployDirectory(), "swerve");
      m_swerveDrive = new SwerveParser(directory).createSwerveDrive(DriveConstants.MAX_SPEED);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    m_swerveDrive.setHeadingCorrection(false);
    m_swerveDrive.setCosineCompensator(false);  // Disabled for simulation
    m_swerveDrive.setAngularVelocityCompensation(      
      true,
      true,
      0.1); // Invert if too much skew occurs as angular velocity increases
    m_swerveDrive.setModuleEncoderAutoSynchronize(false, 3);  // Resynchronize absolute encoders and motor encoders periodically when not moving
    
    m_swerveDrive.pushOffsetsToEncoders();  // Set the absolute encoder to be used over the internal encoder and push the offsets onto it. Throws warning if not possible

    setupPathPlanner();
  }

<<<<<<< Updated upstream
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
=======
  public void setChassisSpeed(ChassisSpeeds desiredSpeeds) {
    
    // Get the desired states of the wheels
    SwerveModuleState[] newStates = kinematics.toSwerveModuleStates(desiredSpeeds);

    frontLeftModule.setState(newStates[0]);
    frontRightModule.setState(newStates[1]);
    backLeftModule.setState(newStates[2]);
    backRightModule.setState(newStates[3]);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   
    // FL, FR, BL, BR
    double loggingStates[] = {
      frontLeftModule.getState().angle.getDegrees(),
      frontLeftModule.getState().speedMetersPerSecond,
      frontRightModule.getState().angle.getDegrees(),
      frontRightModule.getState().speedMetersPerSecond,
      backLeftModule.getState().angle.getDegrees(),
      backLeftModule.getState().speedMetersPerSecond,
      backRightModule.getState().angle.getDegrees(),
      backRightModule.getState().speedMetersPerSecond,
    };


    SmartDashboard.putNumberArray("Swerve Module States", loggingStates);
>>>>>>> Stashed changes
  }

  public void setupPathPlanner(){}

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
      driveFieldOriented(m_swerveDrive.swerveController.getTargetSpeeds(xInput, yInput,
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
      m_swerveDrive.drive(new Translation2d(translationX.getAsDouble() * m_swerveDrive.getMaximumVelocity(),
          translationY.getAsDouble() * m_swerveDrive.getMaximumVelocity()),
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
<<<<<<< Updated upstream
=======


>>>>>>> Stashed changes
}
