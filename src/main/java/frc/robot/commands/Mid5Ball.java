package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AUTO;
import frc.robot.Constants.CONVEYOR;
import frc.robot.Constants.SHOOTER;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.swervelib.SwerveSubsystem;
import frc.swervelib.TrajectoryLogging;

public class Mid5Ball extends SequentialCommandGroup {
    public Mid5Ball(SwerveSubsystem m_swerve, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor, ShooterSubsystem m_shooter) {        
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("MidToB", 1.5, 3.0);
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("MidBToC", 2.0, 3.0);
        PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("MidCToA", 2.0, 3.0);
        
        addCommands(
            new InstantCommand(() -> m_swerve.dt.setKnownState(trajectory1.getInitialState())),

            //turn shooter and adjust hood angle for circle shot
            new InstantCommand(() -> {m_shooter.setRPS(SHOOTER.SETPOINT2, SHOOTER.CIRCLEFF);
                m_shooter.setHoodPosition(SHOOTER.HOODCIRCLE);}, m_shooter),

            // Lower and turn on the intake
            new InstantCommand(() -> m_intake.deployIntake()),

            // Pickup the B ball
            m_swerve.dt.createCommandForTrajectory(trajectory1, m_swerve).raceWith(new TrajectoryLogging(trajectory1, () -> m_swerve.dt.getPose())),

            // Shoot the B ball and preload
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(AUTO.TWOBALLTIME)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor)),
                
            // Pickup the C and HP balls
            m_swerve.dt.createCommandForTrajectory(trajectory2, m_swerve).raceWith(new TrajectoryLogging(trajectory2, () -> m_swerve.dt.getPose())),

            new WaitCommand(2),

            // Go back to the A Ball and pickup that too
            m_swerve.dt.createCommandForTrajectory(trajectory3, m_swerve).raceWith(new TrajectoryLogging(trajectory3, () -> m_swerve.dt.getPose())),

            // Shoot the C, HP, and A balls
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(AUTO.TWOBALLTIME)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor)),

            // Raise and turn off the intake
            new InstantCommand(() -> m_intake.retractIntake()),

            // Turn off shooter
            new InstantCommand(() -> m_shooter.setRPS(0, 0))
        );
    }
}
