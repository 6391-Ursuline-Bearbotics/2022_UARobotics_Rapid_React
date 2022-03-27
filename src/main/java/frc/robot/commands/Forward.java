// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.SwerveSubsystem;

public class Forward extends SequentialCommandGroup {  
  public Forward(SwerveSubsystem m_swerve) {
    PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("MoveForward", 0.5, 3.0);

    addCommands(
      new InstantCommand(() -> m_swerve.dt.setKnownState(trajectory1.getInitialState())),

      m_swerve.dt.createCommandForTrajectory(trajectory1, m_swerve)
    );
  }
}
