package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.swervelib.SwerveSubsystem;

public class Center5Ball extends SequentialCommandGroup {
    public Center5Ball(SwerveSubsystem m_swerve, IntakeSubsystem m_intake) {        
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("StartToA", 2.0, 3.0);
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("AToBSimple", 2.0, 3.0);
        PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("BToC", 2.0, 3.0);
        PathPlannerTrajectory trajectory4 = PathPlanner.loadPath("CToB", 2.0, 3.0);
        
        addCommands(
            new InstantCommand(() -> m_swerve.dt.setKnownPose(trajectory1.getInitialPose())),

            //turn shooter on to get up to speed
            /* new InstantCommand(() -> {m_shooter.setSetpoint(ShooterConstants.kShooter4);
                m_shooter.enable();}, m_shooter), */

            new InstantCommand(() -> m_intake.toggleIntakeWheels(true)),
                    //.andThen(new InstantCommand(() -> m_intake.toggleIntakePosition(true))),
            m_swerve.dt.createCommandForTrajectory(trajectory1, m_swerve),
            new WaitCommand(2), // Placeholder for shooting 2
            m_swerve.dt.createCommandForTrajectory(trajectory2, m_swerve)
            /* new WaitCommand(1), // Placeholder for shooting 1
            m_swerve.dt.createCommandForTrajectory(trajectory3, m_swerve),
            new WaitCommand(.25), // Placeholder for waiting for the second ball
            m_swerve.dt.createCommandForTrajectory(trajectory4, m_swerve),
            new WaitCommand(2) // Placeholder for shooting 2 */
        );
    }
}
