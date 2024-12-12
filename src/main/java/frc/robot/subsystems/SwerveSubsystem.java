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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

//Finds the files needed for swerve and parses them into the swerve drive command alongside the speed limiter stored in the Constants file
//Any files located in src/main/swerve will be used by the swerve parser

    } catch (Exception e) {
      throw new RuntimeException(e);
    }

//Make sure all required .json files are present in the deploy directory to prevent the robot crashing on launch

    m_moduleStatePublisher = NetworkTableInstance.getDefault()
        .getStructArrayTopic("/SwerveStates", SwerveModuleState.struct).publish();

    m_swerveDrive.setMaximumSpeed(DriveConstants.MAX_SPEED);

  }

  public Pose2d getPose() {
    return m_swerveDrive.getPose();
    // Getter method to obtain the current pose of the robot (rotational and positional)
    // Values provided by odometry (as stated in the getPose function)
  }

  public void resetOdometry(Pose2d initialHolonomicPose) {
    m_swerveDrive.resetOdometry(initialHolonomicPose);
    //Overrides the current position with a new inputted position 
  }

  public ChassisSpeeds getRobotVelocity() {
    return m_swerveDrive.getRobotVelocity();
    //Getter method to obtain the current robot-relative (what does that mean?) velocity (x, y, omega.. aka angular velocity, measured in radians per second)
  }

  public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    m_swerveDrive.setChassisSpeeds(chassisSpeeds);
    //Changes the speed of the robot (this is where the drivetrain speed changes)
    //Used by pathplanner
  }

  public void initializePathPlanner() {
      AutoBuilder.configureHolonomic(
        this::getPose, // Provides robot pose (combination of translation and rotation)
        this::resetOdometry, // Resets odometry (runs when auto has a starting pose)
        this::getRobotVelocity, // Provides chassis velocity
        this::setChassisSpeeds, // Continuously sets the robot's speed during operation 
        new HolonomicPathFollowerConfig(
          Constants.AutonConstants.TRANSLATION_PID,
          Constants.AutonConstants.ANGLE_PID,
          Constants.AutonConstants.MAX_MODULE_SPEED,
          m_swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
          new ReplanningConfig()),
          () -> {
            //Identifies whether our team is on the blue or red alliance
            //Useful for setting position based on the alliance
            var alliance = DriverStation.getAlliance(); 

            //Returns a boolean value which is set based on whether the team is on the red alliance or on the blue alliance
            //Red = true, blue = false                                            
            return (alliance.isPresent()) ? (alliance.get() == DriverStation.Alliance.Red) : (false);},
            this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    states = m_swerveDrive.getStates();

    //putNumber puts the value of a variable specified by the programmer on the SmartDashboard program for the operator to view
    SmartDashboard.putNumber("FL Angle (deg)", states[0].angle.getDegrees()); 
    SmartDashboard.putNumber("FR Angle (deg)", states[1].angle.getDegrees());
    SmartDashboard.putNumber("BL Angle (deg)", states[2].angle.getDegrees());
    SmartDashboard.putNumber("BR Angle (deg)", states[3].angle.getDegrees());
    //Returns angle in degrees from -180 degrees to 180 degrees (-π/2 to π/2 in radians) of each wheel

    SmartDashboard.putNumber("FL Speed (m/s)", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FR Speed (m/s)", states[1].speedMetersPerSecond); 
    SmartDashboard.putNumber("BL Speed (m/s)", states[2].speedMetersPerSecond); 
    SmartDashboard.putNumber("BR Speed (m/s)", states[3].speedMetersPerSecond); 
    //Returns the current speed of the motors (does not include vectors)

    SwerveModuleState[] states = m_swerveDrive.getStates(); //outputs the current module states (this includes its velocity and azimuth)
                                                            //azimuth is the measured deviated angular distance from the north or south direction
                                                            //basically how much the wheels have rotated from its initial position
                                                            //erm watdasigma

    //Displays yaw angle on SmartDashboard
    //Yaw is the rotation around the y-axis (you can think of it as the measurement of the robot turning around)
    SmartDashboard.putNumber("Swerve Yaw", m_swerveDrive.getGyroRotation3d().getZ());


    m_moduleStatePublisher.set(states);
  }

  /**
   * Command to drive the robot using translative values and heading as a
   * setpoint.
   *
   * @param translationX Translation in the X direction.
   * @param translationY Translation in the Y direction.
   * @param headingX     Heading X to calculate angle of the joystick.
   * @param headingY     Heading Y to calculate angle of the joystick.s
   * @return Drive command.
   */
  

    // old
   public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier headingX,
      DoubleSupplier headingY) {
         
    return run(() -> {

/*
      Calculates the angle that the robot needs to turn to face its target position

       θ = atan(x / y)    

       x
      ____ target position           
     |   /       
     |  /      
   y | /
     |/          
     θ
     ^
     target angle
*/     

      SmartDashboard.putNumber("Target robot angle", Math.atan2(headingX.getAsDouble(), headingY.getAsDouble()));

      //inputs are cubed to decrease sensitivity of the joysticks, allowing for a finer control
      //the higher the order -> the finer the control
      //be sure to 
      double xInput = Math.pow(translationX.getAsDouble(), 3); 
      double yInput = Math.pow(translationY.getAsDouble(), 3); 
      

      // Make the robot move
      driveFieldOriented(m_swerveDrive.swerveController.getTargetSpeeds(xInput * 0.1, yInput * 0.1,
          headingX.getAsDouble(),
          headingY.getAsDouble(),
          m_swerveDrive.getYaw().getRadians(),
          m_swerveDrive.getMaximumVelocity()));
    });
  }


  /**
   * NEW
   * Command to drive the robot using translative values and heading as angular
   * velocity.
   *
   * @param translationX     Translation in the X direction.
   * @param translationY     Translation in the Y direction.
   * @param angularRotationX Rotation of the robot to set
   * @return Drive command.
   */

  //Drivecommand uses the translation and heading values to drive the robot
  //includes vertical translation, horizontal translation, and angular velocity

  public Command driveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
      DoubleSupplier angularRotationX) {
    return run(() -> {
      // Make the robot move
      m_swerveDrive.drive(
          new Translation2d(
              translationX.getAsDouble() * m_swerveDrive.getMaximumVelocity(),
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
    //Drives the robot with the desired velocity
    m_swerveDrive.driveFieldOriented(velocity);
    
  }

  /**
   * Gets the current field-relative pose of the robot according to odometry.
   * 
   * @return The current robot pose.
   */

}
