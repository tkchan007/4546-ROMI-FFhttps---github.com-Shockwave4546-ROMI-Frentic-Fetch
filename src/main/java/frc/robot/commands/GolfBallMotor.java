// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class GolfBallMotor extends CommandBase {
  private final Drivetrain m_drive;
  boolean mState;

  /** Creates a new GolfBallMotor. */
  public GolfBallMotor(boolean val, Drivetrain drive) {
    mState = val;
    // drive = RobotContainer.drive;
    m_drive = drive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.golfBallMotor(mState);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.golfBallMotor(mState);
  }

  // Called once the command ends or is interrupted.
  // Don't do anything because we want the State of the Motor to remain whatever is being set to be.
  @Override
  public void end(boolean interrupted) {
    m_drive.golfBallMotor(mState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
