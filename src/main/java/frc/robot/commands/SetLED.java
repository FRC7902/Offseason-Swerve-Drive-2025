// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PicoLEDModule;

public class SetLED extends Command {
  private PicoLEDModule m_pico;
  private int brightness, mode, r, g, b;

  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  public SetLED (PicoLEDModule pico, int brightness_inp, int mode_inp, int r_inp, int g_inp, int b_inp) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pico = pico;
    brightness = brightness_inp;
    mode = mode_inp;
    r = r_inp;
    g = g_inp;
    b = b_inp;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    //System.out.println("button received");

    m_pico.setLED(brightness, mode, r, g, b);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
