// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DriveCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Swerve m_swerve;
  private final Joystick m_driver;
  
  // controller axis values
  private double m_LX = 0.0;
  private double m_LY = 0.0;
  private double m_RX = 0.0;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveCommand(Swerve drivetrain, Joystick driver) {
    m_driver = driver;
    m_swerve = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // fetch joystick axis values
    m_LX = m_driver.getRawAxis(0); // left x axis (strafe)
    m_LY = -m_driver.getRawAxis(1); // left y axis (strafe)
    m_RX = m_driver.getRawAxis(4); // right x axis (rotation)

    // deadzones
    m_LX = ( Math.abs(m_LX) < 0.1 ) ? 0 : m_LX;
    m_LY = ( Math.abs(m_LY) < 0.1 ) ? 0 : m_LY;
    m_RX = ( Math.abs(m_RX) < 0.1 ) ? 0 : m_RX;

    // square joysticks
    m_LX = Math.pow(m_LX, 2) * ( Math.abs(m_LX)/m_LX );
    m_LY = Math.pow(m_LY, 2) * ( Math.abs(m_LY)/m_LY );
    m_RX = Math.pow(m_RX, 2) * ( Math.abs(m_RX)/m_RX );
    
    m_swerve.drive(m_LX, m_LY, m_RX);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_swerve.drive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
