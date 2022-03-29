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

public class Right5Ball3 extends SequentialCommandGroup {
    public Right5Ball3(SwerveSubsystem m_swerve, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor, ShooterSubsystem m_shooter) {        
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("LowerToA", 1.5, 3.0);
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("AToB", 1.5, 3.0);
        PathPlannerTrajectory trajectory3 = PathPlanner.loadPath("BToC", 2.0, 3.0);
        PathPlannerTrajectory trajectory4 = PathPlanner.loadPath("CToB", 2.0, 3.0);
        
        addCommands(
            new InstantCommand(() -> m_swerve.dt.setKnownState(trajectory1.getInitialState())),

            //turn shooter and adjust hood angle for circle shot
            new InstantCommand(() -> {m_shooter.setRPS(SHOOTER.SETPOINT2, SHOOTER.CIRCLEFF);
                m_shooter.setHoodPosition(SHOOTER.HOODCIRCLE);}, m_shooter),

            // Lower and turn on the intake
            new InstantCommand(() -> m_intake.deployIntake()),

            // Pickup the A ball
            m_swerve.dt.createCommandForTrajectory(trajectory1, m_swerve).raceWith(new TrajectoryLogging(trajectory1, () -> m_swerve.dt.getPose())),
                
            // Pickup the B ball
            m_swerve.dt.createCommandForTrajectory(trajectory2, m_swerve).raceWith(new TrajectoryLogging(trajectory2, () -> m_swerve.dt.getPose())),

            // Shoot the A, preload and B balls
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(AUTO.THREEBALLTIME)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor)),

            // Pickup the C and HP balls
            m_swerve.dt.createCommandForTrajectory(trajectory3, m_swerve).raceWith(new TrajectoryLogging(trajectory3, () -> m_swerve.dt.getPose())),

            // Wait for the HP to load the second ball
            new WaitCommand(2),

            // Go back to the circle to shoot
            m_swerve.dt.createCommandForTrajectory(trajectory4, m_swerve).raceWith(new TrajectoryLogging(trajectory4, () -> m_swerve.dt.getPose())),

            // Shoot the C and HP balls
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(AUTO.TWOBALLTIME)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor)),

            // Raise and turn off the intake
            new InstantCommand(() -> m_intake.retractIntake()),

            // Turn off shooter
            new InstantCommand(() -> m_shooter.setRPS(0, 0))
        );
    }
}
