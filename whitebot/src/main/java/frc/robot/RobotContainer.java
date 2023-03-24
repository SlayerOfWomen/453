// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.TankDriveCmd;
import frc.robot.subsystems.TankDriveSubsystem;





public class RobotContainer {

  
  private final TankDriveSubsystem m_tankDriveSubsystem = new TankDriveSubsystem();
  private final TankDriveCmd m_tankDriveCmd = new TankDriveCmd(m_tankDriveSubsystem);
  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_tankDriveSubsystem.setDefaultCommand(m_tankDriveCmd);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create a voltage constraint to ensure we don't accelerate too fast
    var autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(
                DriveConstants.ks,
                DriveConstants.kv,
                DriveConstants.ka),
            DriveConstants.kTankDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(
                DriveConstants.kMaxSpeedMetersPerSecond,
                DriveConstants.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DriveConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // An example trajectory to follow.  All units in meters.
    Trajectory exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these two interior waypoints, making an 's' curve path
            List.of(new Translation2d((1 / DriveConstants.robotConversion), (0.5 / DriveConstants.robotConversion)), new Translation2d((2 / DriveConstants.robotConversion), (-0.5 / DriveConstants.robotConversion))),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d((2 / DriveConstants.robotConversion), 0, new Rotation2d(0)),
            // Pass config
            config);

    RamseteCommand ramseteCommand =
        new RamseteCommand(
            exampleTrajectory,
            m_tankDriveSubsystem::getPose,
            new RamseteController(DriveConstants.kRamseteB, DriveConstants.kRamseteZeta),
            new SimpleMotorFeedforward(
                DriveConstants.ks,
                DriveConstants.kv,
                DriveConstants.ka),
            DriveConstants.kTankDriveKinematics,
            m_tankDriveSubsystem::getWheelSpeeds,
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            new PIDController(DriveConstants.kPDriveVel, 0, 0),
            // RamseteCommand passes volts to the callback
            m_tankDriveSubsystem::tankDriveVolts,
            m_tankDriveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    m_tankDriveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return ramseteCommand.andThen(() -> m_tankDriveSubsystem.tankDriveVolts(0, 0));
  }
}

