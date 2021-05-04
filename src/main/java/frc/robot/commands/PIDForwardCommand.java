// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;

public class PIDForwardCommand extends CommandBase {

    private Drivetrain m_drive;
  
    private final double m_avg_distance;
    private double m_cur_distance;
    private double m_heading;
    private double m_cur_heading;
    // positive makes the motors spin forward, negative makes them spin backwards
    private final double m_direction_mult;
    // the average speed we want between the motors, from zero to 1
    private final double m_avg_speed;
    // the difference between the speed of each motor to keep the robot moving straight
    private double m_speed_diff;
    // the maximum allowable speed difference between the motors
    private final double k_max_speed_diff = 0.1;
  
    // Integrator term to drive the difference between the L and R encoders to zero.
    // This accounts for historical inaccuracy; the longer the robot has been curving,
    // the more this part of the control loop will try to correct it
    private double m_distance_int;
    private final double k_distance_int_gain = 0.007;
    // Proportional term to drive the difference between the L and R encoders to zero.
    // This accounts for instantaneous inaccuracy; the worse the robot is currently curving,
    // the more this part of the control loop will try to correct it.
    private double m_distance_prop;
    private final double k_distance_prop_gain = 0.2;
    
  public PIDForwardCommand(Drivetrain drive, double distance, double speed) {
    m_drive = drive;

    // figure out if we're going forwards or backwards
    m_avg_distance = distance;
    if(m_avg_distance > 0) {
      m_direction_mult = 1;
    } else if(m_avg_distance < 0) {
      m_direction_mult = -1;
    } else {
      m_direction_mult = 0;
    }

    // make sure our speed isn't too high or low, otherwise we might try to drive
    // the motors at more than 100% or less than 0% speed
    if(speed < k_max_speed_diff / 2) {
      m_avg_speed = k_max_speed_diff / 2;
    } else if(speed > 1 - (k_max_speed_diff / 2)) {
      m_avg_speed = 1 - (k_max_speed_diff / 2);
    } else {
      m_avg_speed = speed;
    }

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.resetEncoders();
    // we have to initialize these variables, and there's no error yet
    m_distance_int = 0;
    m_distance_prop = 0;
    m_cur_distance = 0;
    m_heading = m_drive.getGyroAngleZ();
    SmartDashboard.putNumber("Forward Distance:", m_avg_distance);
    SmartDashboard.putNumber("Forward Heading:", m_heading);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // calculate the encoder difference to determine proportional error
    m_distance_prop = m_drive.getLeftDistanceMeter() - m_drive.getRightDistanceMeter();
    // accumulate the encoder difference to determine integral error
    m_distance_int += m_distance_prop;
    // calculate the speed difference required to fix any mismatch between the encoders
    // using instantaneous (proportional) and historical (integral) error data
    m_speed_diff =  (m_distance_prop * k_distance_prop_gain) + (m_distance_int * k_distance_int_gain);

    // Make sure we're not over our motor speed difference limit
    if(m_speed_diff > k_max_speed_diff) {
      m_speed_diff = k_max_speed_diff;
    } else if(m_speed_diff < -1 * k_max_speed_diff) {
      m_speed_diff = -1 * k_max_speed_diff;
    }

    // set the speed of each motor using our drive subsystem
    // m_drive.motorSpeed( (m_avg_speed - (m_speed_diff / 2)) * m_direction_mult, (m_avg_speed + (m_speed_diff / 2)) * m_direction_mult);
    m_cur_heading = m_drive.getGyroAngleZ();
    // If Negative Turning While going forward
    if (m_direction_mult * m_cur_heading > (m_heading + 1.0)) {
        m_drive.motorSpeed( 
            (m_avg_speed - (m_speed_diff / 2)) * m_direction_mult, 
            (m_avg_speed + (m_speed_diff / 2)) * m_direction_mult
        );
    } else if (m_direction_mult * m_cur_heading < (m_heading + 1.0)) {
        m_drive.motorSpeed( 
            (m_avg_speed + (m_speed_diff / 2)) * m_direction_mult, 
            (m_avg_speed - (m_speed_diff / 2)) * m_direction_mult
        );
    } else {
        m_drive.motorSpeed( 
            (m_avg_speed) * m_direction_mult, 
            (m_avg_speed) * m_direction_mult
        );
    }

    m_cur_distance = Math.abs((m_drive.getLeftDistanceMeter() + m_drive.getRightDistanceMeter())/2);
    SmartDashboard.putNumber("Current Distance:", m_cur_distance);
    SmartDashboard.putNumber("Current Heading:", m_cur_heading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.motorSpeed(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we're done once the average of the two encoders is the specified distance
    return m_cur_distance >= Math.abs(m_avg_distance);
  }
}
