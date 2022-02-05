package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.SwerveSubsystem;

public class Center5Ball extends SequentialCommandGroup {
    public Center5Ball(SwerveSubsystem m_swerve) {        
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("MoveForward", 2.0, 3.0);
        
        addCommands(
            new InstantCommand(() -> m_swerve.dt.setKnownPose(trajectory1.getInitialPose())),

            //turn shooter on to get up to speed
            /* new InstantCommand(() -> {m_shooter.setSetpoint(ShooterConstants.kShooter4);
                m_shooter.enable();}, m_shooter), */

            //back up 1 meter
            m_swerve.dt
        );
    }
}
