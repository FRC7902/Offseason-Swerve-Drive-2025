// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AbsoluteFieldDriveCommand extends Command {
  private final SwerveSubsystem m_swerveSubsystem;
  private final DoubleSupplier vX, vY, heading;

  /** Creates a new AbsoluteFieldDriveCommand. */
  public AbsoluteFieldDriveCommand(SwerveSubsystem swerve, DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier heading) {
    this.m_swerveSubsystem = swerve;
    this.vX = vX;
    this.vY = vY;
    this.heading = heading;

    addRequirements(m_swerveSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

      // Get desired chassis speeds based on a 2 joystick module
      /* ChassisSpeeds desiredSpeeds = m_swerveSubsystem.getTargetSpeeds(
        vX.getAsDouble(),
        vY.getAsDouble(),
        new Rotation2d(heading.getAsDouble() * Math.PI)); */

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
