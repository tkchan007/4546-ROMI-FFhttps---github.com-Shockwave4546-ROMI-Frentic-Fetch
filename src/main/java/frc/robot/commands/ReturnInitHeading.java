// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ReturnInitHeading extends CommandBase {

  private final Drivetrain m_drive;
  private double m_heading;
  private double c_heading;
  private double e_heading;
  private double m_speed = 0.35;

  /** Creates a new ReturnInitHeading. */
  public ReturnInitHeading(Drivetrain drive) {
    m_drive = drive;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // grab the initial heading
    m_heading = m_drive.getInitHeading();
    if (m_heading < 0) {
      e_heading = 360.0 + m_heading;
    } else {
      e_heading = m_heading;
    }
    // Get current heading
    c_heading = m_drive.getGyroAngleZ();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get current heading
    c_heading = m_drive.getGyroAngleZ();
    if (c_heading < e_heading) {
      m_drive.motorSpeed(m_speed, -1 * m_speed);
    } else {
      m_drive.motorSpeed(-1 * m_speed, m_speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcadeDrive(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Starts at negative angle reading
    return (c_heading < e_heading + 0.1) && (c_heading > e_heading - 0.1);
  }

}
