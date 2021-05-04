// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
  // Drive Base Constants - Going to use the PIDController to improve accuracy
  public static final class DriveBase {
    public static final double ksVolts = 0.895; // 0.929; // 0.895;
    public static final double kvVoltSecondsPerMeter = 6.73; // 6.73;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0136; // 0.0389; // 0.0136;

    public static final double kPDriveVel = 0.021; // 0.021;

    public static final double kTrackwidthMeters = 0.142; // 0.142072613;
    public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter, kaVoltSecondsSquaredPerMeter);
    // PIDController - kP, kD and kI
    public static final double kP = 0.75;
    public static final double kD = 0;

    public static final PIDController m_leftPIDController = new PIDController(kP, 0, 0);
    public static final PIDController m_rightPIDController = new PIDController(kP, 0, 0);

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 0.8; // 0.8;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.8;

    // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
  }
}
