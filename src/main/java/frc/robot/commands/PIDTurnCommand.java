// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDTurnCommand extends CommandBase {

  private Drivetrain m_drive;
  private double m_degrees;
  private double m_tolerance;

  private double kP = Constants.DriveBase.kP;
  private double kD = Constants.DriveBase.kD;

  private double error;
  private double last_error;
  private double derivative = 0;

  public PIDTurnCommand(Drivetrain drive, double degrees, double tolerance) {
    m_degrees = degrees;
    m_drive = drive;
    m_tolerance = tolerance;

    SmartDashboard.putNumber("Req. Degree:", degrees);
    SmartDashboard.putNumber("Tolerance:", m_tolerance);
    // Requirements
    addRequirements(m_drive);
  }

@Override
  public void initialize() {
    // Clear the Gyro 1st
    m_drive.resetGyro();
    // Now, set the initial error as the delta between the current heading and the degrees to turn
    error = m_degrees - m_drive.getHeading();

    SmartDashboard.putNumber("Initial Heading:", m_drive.getHeading());
    SmartDashboard.putNumber("Initial Error:", error);
  }

  @Override
  public void execute() {
    // Setup to get the delta time to use to setup the derivative term
    double startTime = Timer.getFPGATimestamp();
    // Feedback from the error in PID loop
    double errfeedback = Math.abs((error * kP) + (derivative * kD));
    // drive using the correctional term
    m_drive.arcadeDrive(0, errfeedback * Math.round(Math.abs(error)/error));
/*
    if (error > 0) {
      m_drive.arcadeDrive(0, -errfeedback);
    } else {
      m_drive.arcadeDrive(0, errfeedback);
    }
*/
    // Prepare for the next loop
    last_error = error;
    error = m_degrees - m_drive.getHeading();
    // derivative term using the delta error and the delta time muliply by the kD term
    derivative = (error - last_error) / (Timer.getFPGATimestamp() - startTime) * kD;

    SmartDashboard.putNumber("Runtime Heading:", m_drive.getHeading());
    SmartDashboard.putNumber("Runtime Error:", error);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop moving
    m_drive.arcadeDrive(0, 0);
    SmartDashboard.putNumber("End Angle: ", m_drive.getHeading());
  }

  @Override
  public boolean isFinished() {
    // As error (current loop calc degree turned) gets closer to the tolerance (absolute tolerance), can exit out of the PIDTurn
    return Math.abs(error) <= m_tolerance;
  }
}
