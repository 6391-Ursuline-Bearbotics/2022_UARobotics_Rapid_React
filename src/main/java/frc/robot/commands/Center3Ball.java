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

public class Center3Ball extends SequentialCommandGroup {
    public Center3Ball(SwerveSubsystem m_swerve, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor, ShooterSubsystem m_shooter) {        
        PathPlannerTrajectory trajectory1 = PathPlanner.loadPath("CenterToA", 2.0, 3.0);
        PathPlannerTrajectory trajectory2 = PathPlanner.loadPath("AToB", 2.0, 3.0);
        
        addCommands(
            new InstantCommand(() -> m_swerve.dt.setKnownState(trajectory1.getInitialState())),

            // shooter speed and adjust hood angle for circle shot
            new InstantCommand(() -> {m_shooter.setRPS(SHOOTER.SETPOINT2, SHOOTER.CIRCLEFF);
                m_shooter.setHoodPosition(SHOOTER.HOODCIRCLE);}, m_shooter),

            // Shoot the preload
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(1)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor)),
        
            // Lower and turn on the intake
            new InstantCommand(() -> m_intake.toggleIntake(true))
                .andThen(new InstantCommand(() -> m_intake.toggleIntakePosition(true))),

            // Pickup the A ball
            m_swerve.dt.createCommandForTrajectory(trajectory1, m_swerve),

            // Pickup the B ball
            m_swerve.dt.createCommandForTrajectory(trajectory2, m_swerve),

            // Shoot the A & B ball
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(2)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor))
            
/*             // Pick up the C and HP balls
            m_swerve.dt.createCommandForTrajectory(trajectory3, m_swerve),

            // Waiting for the second ball
            new WaitCommand(.25),

            // Back to the same spot we shot the B ball
            m_swerve.dt.createCommandForTrajectory(trajectory4, m_swerve),

            // Shoot the B ball
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(5)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor)) */
        );
    }
}
