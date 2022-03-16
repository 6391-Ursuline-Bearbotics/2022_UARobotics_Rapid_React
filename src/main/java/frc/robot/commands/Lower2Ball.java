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

public class Lower2Ball extends SequentialCommandGroup {
    public Lower2Ball(SwerveSubsystem m_swerve, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor, ShooterSubsystem m_shooter) {        
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("LowerToA", 2.0, 3.0);
        
        addCommands(
            new InstantCommand(() -> m_swerve.dt.setKnownState(trajectory1.getInitialState())),

            //turn shooter and adjust hood angle for circle shot
            new InstantCommand(() -> {m_shooter.setRPS(SHOOTER.SETPOINT2, SHOOTER.CIRCLEFF);
                m_shooter.setHoodPosition(SHOOTER.HOODCIRCLE);}, m_shooter),

            // Lower and turn on the intake
            new InstantCommand(() -> m_intake.toggleIntakeWheels(true))
                .andThen(new InstantCommand(() -> m_intake.toggleIntakePosition(true))),

            // Pickup the A ball
            m_swerve.dt.createCommandForTrajectory(trajectory1, m_swerve).raceWith(new TrajectoryLogging(trajectory1, () -> m_swerve.dt.getPose())),

            // Shoot the preload and A ball
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(2)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor))
        );
    }
}
