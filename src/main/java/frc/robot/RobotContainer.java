/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

// WPI Imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.commands.AutoAim;
import frc.robot.commands.Center5Ball;
// Command Imports
import frc.robot.commands.NextClimbPosition;
// Subsystem Imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.PhotonVision;
// Constant Imports
import frc.robot.Constants.OI;
// Special Imports
import frc.robot.UA6391.XboxController6391;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final PhotonVision m_PhotonVision = new PhotonVision();
  
  private static SwerveDrivetrainModel dt;
  private static SwerveSubsystem m_swerveSubsystem;
  //@Log
  //public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  @Log
  public final LEDSubsystem m_LED;
  @Log
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  //@Log
  //public final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
  //@Log
  //public final ClimbSubsystem m_climb = ClimbSubsystem.Create();

  private final Center5Ball center5;
  
  @Log(tabName = "Dashboard")
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // The driver's controller
  XboxController6391 drv = new XboxController6391(OI.DRVCONTROLLERPORT, 0.1);
  XboxControllerSim m_driverControllerSim = new XboxControllerSim(OI.DRVCONTROLLERPORT);
  private final ControlScheme m_scheme = new ControlScheme(drv.getXboxController());

  // The operator's controller
  XboxController6391 op = new XboxController6391(OI.OPCONTROLLERPORT, 0.1);
  XboxControllerSim m_operatorControllerSim = new XboxControllerSim(OI.OPCONTROLLERPORT);

/*   Button frontConveyorSensor = new Button(() -> m_conveyor.getFrontConveyor());
  Button topConveyorSensor = new Button(() -> m_conveyor.getTopConveyor());
  Button shooteratsetpoint = new Button(() -> m_shooter.atSetpoint()); */

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    dt = BearSwerveHelper.createBearSwerve();
    m_swerveSubsystem = BearSwerveHelper.createSwerveSubsystem(dt);
    m_LED = new LEDSubsystem(m_PhotonVision, dt);
    center5 = new Center5Ball(m_swerveSubsystem, m_intake);

    m_swerveSubsystem.setDefaultCommand(new RunCommand(() -> dt.setModuleStates(m_scheme.getJoystickSpeeds()), m_swerveSubsystem));

    m_PhotonVision.fieldSetup(m_swerveSubsystem.dt.getField());

    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(OI.PRACTICE);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Constantly checks to see if the intake motor has stalled
    m_intake.setDefaultCommand(new RunCommand(m_intake::checkStall, m_intake));

/*     m_climb.setDefaultCommand(
      // Use right y axis to control the speed of the climber
      new RunCommand(
        () -> m_climb
          .setOutput(Math.max(op.TriggerL(), drv.TriggerL()),
            Math.max(op.TriggerR(), drv.TriggerR())), m_climb)); */

    autoChooser.setDefaultOption("Center5", center5);
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Spin up the shooter to far trench speed when the 'X' button is pressed.
/*     drv.XButton.or(op.XButton)
      .whenActive(new InstantCommand(() -> {
        m_shooter.setRPS(3.0);
      }, m_shooter));

    // Stop the Shooter when the B button is pressed
    drv.YButton.or(op.YButton)
      .whenActive(new InstantCommand(() -> {
        m_shooter.setRPS(0);
      }, m_shooter)); */
    
    // While driver holds the A button Auto Aim to the High Hub using the left stick for distance control
    drv.AButton.whileActiveOnce(new AutoAim(m_swerveSubsystem, m_PhotonVision, true, m_scheme));

    // While driver holds the B button Auto Aim to the closest ball using the left stick for distance control
    drv.BButton.whileActiveOnce(new AutoAim(m_swerveSubsystem, m_PhotonVision, false, m_scheme));

    // Turn on the conveyor when:
    // the A button is pressed (either controller) and either the top sensor is not blocked or the shooter is up to speed
    // if the bottom sensor is blocked (ball waiting to go up) unless top sensor blocked (the ball has no place to go)
/*     (topConveyorSensor.negate()
      .and(frontConveyorSensor))
    .or(drv.AButton.and(shooteratsetpoint.or(topConveyorSensor.negate())))
    .or(op.AButton.and(shooteratsetpoint.or(topConveyorSensor.negate())))
    .whenActive(new InstantCommand(m_conveyor::turnOn, m_conveyor))
    .whenInactive(new InstantCommand(m_conveyor::turnOff, m_conveyor)); */

    // When right bumper is pressed raise/lower the intake and stop/start the intake on both controllers
    drv.BumperR.or(op.BumperR).whenActive(new InstantCommand(() -> m_intake.toggleIntakeWheels(true))
      .andThen(new InstantCommand(() -> m_intake.toggleIntakePosition(true))));
    
    // When the left bumper is pressed on either controller right joystick is super slow turn
    /* drv.BumperL.or(op.BumperL).whileActiveOnce(new InstantCommand(() -> m_robotDrive.setMaxDriveOutput(
        DriveConstants.kMaxOutputForwardSlow, DriveConstants.kMaxOutputRotationSlow)))
      .whenInactive(new InstantCommand(() -> m_robotDrive.setMaxDriveOutput(
        DriveConstants.kMaxOutputForward, DriveConstants.kMaxOutputRotation))); */
    //drv.BumperL.whileActiveOnce(m_robotDrive.driveStraight(() -> -drv.JoystickLY()));

    // When the back button is pressed run the conveyor backwards until released
/*     drv.BackButton.or(op.BackButton).whenActive(new InstantCommand(m_conveyor::turnBackwards, m_conveyor))
      .whenInactive(new InstantCommand(m_conveyor::turnOff, m_conveyor)); */
    
    // When start button is pressed for at least a second advance to the next climb stage
    //drv.StartButton.or(op.StartButton).whileActiveOnce(new WaitCommand(0.5).andThen(new NextClimbPosition(m_climb).withTimeout(5)));

    // Create "button" from POV Hat in up direction.  Use both of the angles to the left and right also.
    //drv.POVUp.whileActiveOnce(new LStoCP(m_shooter, m_robotDrive, m_intake));
/*     drv.POVUp.whenActive(() -> m_shooter.setRPS(ShooterConstants.kShooter1));
    drv.POVRight.whenActive(() -> m_shooter.setRPS(ShooterConstants.kShooter2));
    drv.POVDown.whenActive(() -> m_shooter.setRPS(ShooterConstants.kShooter3));
    drv.POVLeft.whenActive(() -> m_shooter.setRPS(ShooterConstants.kShooter4));

    op.POVUp.whenActive(() -> m_shooter.setRPS(ShooterConstants.kShooter1));
    op.POVRight.whenActive(() -> m_shooter.setRPS(ShooterConstants.kShooter2));
    op.POVDown.whenActive(() -> m_shooter.setRPS(ShooterConstants.kShooter3));
    op.POVLeft.whenActive(() -> m_shooter.setRPS(ShooterConstants.kShooter4)); */

/*     op.POVRight.whenActive(new InstantCommand(m_conveyor::CPRightSlow, m_conveyor))
      .whenInactive(new InstantCommand(m_conveyor::CPOff, m_conveyor));
  
    op.POVLeft.whenActive(new InstantCommand(m_conveyor::CPLeftSlow, m_conveyor))
      .whenInactive(new InstantCommand(m_conveyor::CPOff, m_conveyor));

    op.POVUp.whenActive(new StartEndCommand(m_conveyor::CPOn, m_conveyor::CPOff, m_conveyor).withTimeout(4)); */

    // Create "button" from POV Hat in down direction.  Use both of the angles to the left and right also.
    //drv.POVDown.whileActiveOnce(new CPtoLS(m_shooter, m_robotDrive, m_intake));

    // POV Up Direction on Operator Controller relatively increases the current setpoint of the shooter
    //op.POVUpish.whenActive(new InstantCommand(() -> {m_shooter.setRPS(m_shooter.getSetpoint() + 1);}));

    // POV Down Direction on Operator Controller relatively increases the current setpoint of the shooter
    //op.POVDownish.whenActive(new InstantCommand(() -> {m_shooter.setRPS(m_shooter.getSetpoint() - 1);}));

    // Add a light rumble when there is a ball at the bottom of the conveyor moving up.
    /* topConveyorSensor.negate().and(frontConveyorSensor)
        .whenActive(() -> {drv.setLeftRumble(0.3); drv.setRightRumble(0.3);})
        .whenInactive(() -> {drv.setLeftRumble(0); drv.setRightRumble(0);}); */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
