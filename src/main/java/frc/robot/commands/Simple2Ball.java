package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ControlScheme;
import frc.robot.Constants.CONVEYOR;
import frc.robot.Constants.SHOOTER;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.subsystems.ShooterSubsystem;
import frc.swervelib.SwerveSubsystem;

public class Simple2Ball extends SequentialCommandGroup {
    public Simple2Ball(SwerveSubsystem m_swerve, IntakeSubsystem m_intake, ConveyorSubsystem m_conveyor, ShooterSubsystem m_shooter) { //, PhotonVision pv) {
        
        addCommands(
            //turn shooter on and adjust hood angle for circle shot
            new InstantCommand(() -> {m_shooter.setRPS(SHOOTER.SETPOINT2, SHOOTER.CIRCLEFF);
                m_shooter.setHoodPosition(SHOOTER.HOODCIRCLE);}, m_shooter),

            // Lower and turn on the intake
            new InstantCommand(() -> m_intake.toggleIntakeWheels(true))
                .andThen(new InstantCommand(() -> m_intake.toggleIntakePosition(true))),

            // Pickup the A ball
            new RunCommand(() -> m_swerve.dt.goToPose(m_swerve.dt.getPose().transformBy(new Transform2d(
                    new Translation2d(1, 0), Rotation2d.fromDegrees(0.0))),
                    m_swerve.dt.getGyroscopeRotation().getDegrees())).withTimeout(2),

            //new AutoAim(m_swerve, pv, true, new ControlScheme(new XboxController(0))),

            // Shoot the preload and A ball
            new RunCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor).withTimeout(2)
                .andThen(new InstantCommand(m_conveyor::turnOff, m_conveyor))
        );
    }
}
