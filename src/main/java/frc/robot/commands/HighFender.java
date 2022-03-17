package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CONVEYOR;
import frc.robot.Constants.SHOOTER;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.swervelib.SwerveSubsystem;
import frc.swervelib.TrajectoryLogging;

public class HighFender extends SequentialCommandGroup {
    public HighFender(SwerveSubsystem m_swerve, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor, ShooterSubsystem m_shooter) {        
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("FenderToD", 2.0, 3.0);
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("AToB", 2.0, 3.0);
        PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("BToC", 2.0, 3.0);
        PathPlannerTrajectory trajectory4 = PathPlanner.loadPath("CToB", 2.0, 3.0);
        
        addCommands(
            //new InstantCommand(() -> m_swerve.dt.setKnownState(trajectory1.getInitialState())),

            //turn shooter and adjust hood angle for circle shot
            new InstantCommand(() -> {m_shooter.setRPS(SHOOTER.SETPOINT2, SHOOTER.FENDERFF);
                m_shooter.setHoodPosition(0);}, m_shooter),

            new WaitCommand(0.5),

            // Shoot the preload
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(1)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor)),

            // Lower and turn on the intake
            new InstantCommand(() -> m_intake.toggleIntakeWheels(true))
                .andThen(new InstantCommand(() -> m_intake.toggleIntakePosition(true))),

            // Pickup the A ball
            m_swerve.dt.createCommandForTrajectory(trajectory1, m_swerve).raceWith(new TrajectoryLogging(trajectory1, () -> m_swerve.dt.getPose())),

            // Shoot the A ball
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(2)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor)),
            
            // Stop shooter
            new InstantCommand(() -> m_shooter.setRPS(0, 0)),

            // Raise Intake
            new InstantCommand(() -> m_intake.toggleIntakeWheels(true))
                .andThen(new InstantCommand(() -> m_intake.toggleIntakePosition(true)))
        );
    }
}
