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
import frc.robot.commands.AutoAimRotate;
import frc.robot.commands.Center5Ball;
// Subsystem Imports
import frc.robot.subsystems.ShooterSubsystem;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.PhotonVision;
import frc.robot.Constants.CONVEYOR;
import frc.robot.Constants.DRIVE;
// Constant Imports
import frc.robot.Constants.OI;
import frc.robot.Constants.SHOOTER;
// Special Imports
import frc.robot.UA6391.XboxController6391;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //public final PhotonVision m_PhotonVision = new PhotonVision();
  
  public static SwerveDrivetrainModel dt;
  public static SwerveSubsystem m_swerveSubsystem;
  @Log
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  //@Log
  //public final LEDSubsystem m_LED;
  @Log
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  @Log
  public final ConveyorSubsystem m_conveyor = new ConveyorSubsystem();
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

  Button frontConveyorSensor = new Button(() -> m_conveyor.getFrontConveyor());
  Button topConveyorSensor = new Button(() -> m_conveyor.getTopConveyor());
  Button shooteratsetpoint = new Button(() -> m_shooter.atSetpoint());

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    dt = BearSwerveHelper.createBearSwerve();
    m_swerveSubsystem = BearSwerveHelper.createSwerveSubsystem(dt);
    //m_LED = new LEDSubsystem(m_PhotonVision, dt);
    center5 = new Center5Ball(m_swerveSubsystem, m_intake);

    m_swerveSubsystem.setDefaultCommand(new RunCommand(() -> dt.setModuleStates(m_scheme.getJoystickSpeeds()), m_swerveSubsystem));

    //m_PhotonVision.fieldSetup(m_swerveSubsystem.dt.getField());

    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(OI.PRACTICE);

    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Constantly checks to see if the intake motor has stalled
    //m_intake.setDefaultCommand(new RunCommand(m_intake::checkStall, m_intake));

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
    // While driver holds the A button Auto Aim to the High Hub using the left stick for distance control
    //drv.AButton.whileActiveOnce(new AutoAim(m_swerveSubsystem, m_PhotonVision, true, m_scheme));

    // While driver holds the B button Auto Aim to the closest ball using the left stick for distance control
    //drv.BButton.whileActiveOnce(new AutoAim(m_swerveSubsystem, m_PhotonVision, false, m_scheme));

    //drv.XButton.whileActiveOnce(new AutoAimRotate(m_swerveSubsystem, m_PhotonVision, true, m_scheme));

    //drv.YButton.whileActiveOnce(new AutoAimRotate(m_swerveSubsystem, m_PhotonVision, true, m_scheme));

    // When the left bumper is pressed on either controller right joystick is super slow turn
/*     drv.BumperL.whileActiveOnce(new InstantCommand(() -> m_swerveSubsystem.dt.setMaxSpeeds(
        DRIVE.MAX_FWD_REV_SPEED_MPS_SLOW, DRIVE.MAX_STRAFE_SPEED_MPS_SLOW, DRIVE.MAX_ROTATE_SPEED_RAD_PER_SEC_SLOW)))
      .whenInactive(new InstantCommand(() -> m_swerveSubsystem.dt.setMaxSpeeds(
        DRIVE.MAX_FWD_REV_SPEED_MPS, DRIVE.MAX_STRAFE_SPEED_MPS, DRIVE.MAX_ROTATE_SPEED_RAD_PER_SEC))); */
  
    // When start button is pressed reorient the field drive to the current heading
    drv.StartButton.whileActiveOnce(new InstantCommand(() -> dt.zeroGyroscope()));

    // Turn on the conveyor when the bottom sensor is blocked (ball waiting to go up)
    // unless top sensor blocked (the ball has no place to go)
    (topConveyorSensor.and(frontConveyorSensor.negate()))
    .whenActive(new InstantCommand(() -> {
          m_conveyor.on(CONVEYOR.SPEED);
          drv.setLeftRumble(0.3);
          drv.setRightRumble(0.3);}, m_conveyor))
    .whenInactive(new InstantCommand(() -> {
          m_conveyor.turnOff();
          drv.setLeftRumble(0);
          drv.setRightRumble(0);}, m_conveyor));
    
    // Turn on the conveyor while held then off
    op.AButton.and(shooteratsetpoint.or(topConveyorSensor.negate()))
    .whenActive(new InstantCommand(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED), m_conveyor))
    .whenInactive(new InstantCommand(m_conveyor::turnOff, m_conveyor));

    // Spin the conveyor backwards when held
    op.BButton.whenActive(new InstantCommand(() -> m_conveyor.on(CONVEYOR.BACKSPEED), m_conveyor))
        .whenInactive(new InstantCommand(() -> m_conveyor.turnOff(), m_conveyor));

    // When right bumper is pressed raise/lower the intake and stop/start the intake on both controllers
    op.BumperR.whenActive(new InstantCommand(() -> m_intake.toggleIntakeWheels(true))
      .andThen(new InstantCommand(() -> m_intake.toggleIntakePosition(true))));

    // Spin up the shooter for the fender shot when the 'X' button is pressed.
    op.XButton.whenActive(new InstantCommand(() -> {
      m_shooter.setRPS(39.0, SHOOTER.FENDERFF);
      m_shooter.setHoodPosition(0);
    }, m_shooter));
  
    // Stop the Shooter when the Y button is pressed
    op.YButton.whenActive(new InstantCommand(() -> {
        m_shooter.setRPS(0, 0);
      }, m_shooter));
    
    // Set the hood and shooter speed for the distance from the circle of balls around the hub
    op.POVRight.whenActive(() -> {m_shooter.setRPS(SHOOTER.SETPOINT2, SHOOTER.CIRCLEFF);
          m_shooter.setHoodPosition(SHOOTER.HOODCIRCLE);});
    // Set the hood and shooter speed for a low goal shot against the fender.
    op.POVLeft.whenActive(() -> {m_shooter.setRPS(SHOOTER.SETPOINT4, SHOOTER.LOWFF);
          m_shooter.setHoodPosition(SHOOTER.HOODLOW);});

    // Change the speed up or down of the current shot
    op.POVUp.whenActive(() -> m_shooter.relativeSpeedChange(SHOOTER.SPEEDCHANGE));
    op.POVDown.whenActive(() -> m_shooter.relativeSpeedChange(-SHOOTER.SPEEDCHANGE));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
