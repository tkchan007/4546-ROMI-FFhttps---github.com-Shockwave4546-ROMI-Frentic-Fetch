// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    addCommands(
        // new DriveDistance(0.5, 0.508, drivetrain),
        // PID Forward Command - drivetrain, distance, speed - distance is in METER 0.5 - 20inches, how fast to drive it
        //
        new InstantCommand(drivetrain::setInitHeading),
        // Up - 1st
        new PIDForwardCommand(drivetrain, 0.3, 0.72),  // 0.3 = 12" - 0.3
        new PauseCommand(drivetrain),
        new TurnDegrees(-0.4, 82.05, drivetrain),   // 85.65
        new PauseCommand(drivetrain),
        new GolfBallMotor(true, drivetrain),
        new PauseCommand(drivetrain),
        new GolfBallMotor(true, drivetrain),
        new PauseCommand(drivetrain),
        new PIDForwardCommand(drivetrain, 0.3, 0.72), // 0.4 = 16"
        //
        // Up-Turn and Back Down 
        new PauseCommand(drivetrain),
        new TurnDegrees(-0.4, 5.5, drivetrain), // 4 
        new PauseCommand(drivetrain),
        new PIDForwardCommand(drivetrain, -0.72, 0.72),
        //
        // Turn and Forward (H)
        new PauseCommand(drivetrain),
        new TurnDegrees(0.4, 76.0, drivetrain), // 91.0 - 72
        // new ReturnInitHeading(drivetrain),
        new PauseCommand(drivetrain),
        new PIDForwardCommand(drivetrain, 0.285, 0.72),  // 0.3
        //
        // Turn and Up for 2nd ball
        new PauseCommand(drivetrain),
        new TurnDegrees(-0.4, 80.25, drivetrain),     // 84.75
        new PauseCommand(drivetrain),
        new PauseCommand(drivetrain),
        new PIDForwardCommand(drivetrain, 0.68, 0.72), // 0.4 = 16" -> 0.65 to 0.68
        //
        // Turn and then come back down...
        new PauseCommand(drivetrain),
        new TurnDegrees(0.3, 1.2, drivetrain),    // 3.5, 1.75 3.2
        new PauseCommand(drivetrain),
        new PIDForwardCommand(drivetrain, -0.69, 0.72),  // speed from 7 to 6 -0.67 to -0.69
        //
        new PauseCommand(drivetrain),
        new TurnDegrees(0.4, 82.5, drivetrain),   // 68.2 
        // new ReturnInitHeading(drivetrain),
        new PauseCommand(drivetrain),
        new PIDForwardCommand(drivetrain, 0.54, 0.72),
        //
        //
        new PauseCommand(drivetrain),
        new TurnDegrees(-0.4, 76.12, drivetrain),    // 84.2, 82.25
        new PauseCommand(drivetrain),
        new PIDForwardCommand(drivetrain, 0.6, 0.72),
// */
        new PauseCommand(drivetrain),
        new PIDForwardCommand(drivetrain, -0.3, 0.72),
        new PauseCommand(drivetrain),
        new TurnDegrees(-0.4, 82.45, drivetrain),

        //
        new PIDForwardCommand(drivetrain, -0.42, 0.72),
        new PauseCommand(drivetrain)
        // PID Turn Command - drivetrain, degrees, tolerance
      //  new TurnDegrees(0.4, 180, drivetrain),
      //  new PauseCommand(drivetrain)
        // new PIDForwardCommand(drivetrain, 0.5, 0.7)
        // new PIDTurnCommand(drivetrain, 180, 0.3),
        // new DriveDistance(0.5, 0.508, drivetrain),
        // new PIDForwardCommand(drivetrain, 0.5, 0.7),
        // new TurnDegrees(0.5, 180, drivetrain));
        // new PIDTurnCommand(drivetrain, 180, 0.3)
        );
      }
}
