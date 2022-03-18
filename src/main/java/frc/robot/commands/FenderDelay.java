package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
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

public class FenderDelay extends SequentialCommandGroup {
    public FenderDelay(SwerveSubsystem m_swerve, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor, ShooterSubsystem m_shooter) {        
        addCommands(
            //turn shooter and adjust hood angle for fender shot
            new InstantCommand(() -> {m_shooter.setRPS(SHOOTER.SETPOINT2, SHOOTER.FENDERFF);
                m_shooter.setHoodPosition(0);}, m_shooter),

            new WaitCommand(1),

            // Shoot the preload
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(1)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor)),

            // Turn shooter off
            new InstantCommand(() -> m_shooter.setRPS(0, 0)),

            new WaitCommand(5),

            // Drive forwards at .5 meters per second
            new RunCommand(() -> m_swerve.dt.setModuleStates(new ChassisSpeeds(0.5, 0, 0)), m_swerve)
        );
    }
}
