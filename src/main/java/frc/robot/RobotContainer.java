/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
// WPI Imports
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import frc.robot.commands.AutoAim;
import frc.robot.commands.AutoAimRotate;
import frc.robot.commands.Center3Ball;
import frc.robot.commands.FenderDelay;
import frc.robot.commands.Forward;
import frc.robot.commands.Left2BallD;
import frc.robot.commands.Mid4Ball;
import frc.robot.commands.Mid5Ball;
import frc.robot.commands.Right5Ball;
import frc.robot.commands.SemiCircle;
import frc.robot.commands.Simple2Ball;
import frc.robot.commands.Straight;
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
import frc.robot.Constants.INTAKE;
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
  public final PhotonVision m_PhotonVision = new PhotonVision();
  
  public static SwerveDrivetrainModel dt;
  public static SwerveSubsystem m_swerveSubsystem;
  @Log
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  //@Log
  //public final LEDSubsystem m_LED;
  @Log
  public final IntakeSubsystem m_intake = new IntakeSubsystem();
  @Log
  public final ClimbSubsystem m_climb = ClimbSubsystem.Create();

  private final Right5Ball right5;
  private final Mid4Ball mid4;
  private final Mid5Ball mid5;
  private final Left2BallD left2;
  private final SemiCircle semicircle;
  private final Simple2Ball simple2;
  private final FenderDelay fenderDelay;
  private final Straight straight;
  private final Forward forward;

  private boolean fast = true;
  
  @Log(tabName = "Dashboard")
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // The driver's controller
  XboxController6391 drv = new XboxController6391(OI.DRVCONTROLLERPORT, 0.1);
  XboxControllerSim m_driverControllerSim = new XboxControllerSim(OI.DRVCONTROLLERPORT);
  private final ControlScheme m_scheme = new ControlScheme(drv);

  // The operator's controller
  XboxController6391 op = new XboxController6391(OI.OPCONTROLLERPORT, 0.1);
  XboxControllerSim m_operatorControllerSim = new XboxControllerSim(OI.OPCONTROLLERPORT);

  @Log
  public final ConveyorSubsystem m_conveyor = new ConveyorSubsystem(drv, op);

  Button frontConveyorSensor = new Button(() -> m_conveyor.getFrontConveyor());
  Button topConveyorSensor = new Button(() -> m_conveyor.getTopConveyor());
  Button shooteratsetpoint = new Button(() -> m_shooter.atSetpoint());
  Button auto = new Button(() -> m_conveyor.getAutoConvey());

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    dt = BearSwerveHelper.createBearSwerve();
    m_swerveSubsystem = BearSwerveHelper.createSwerveSubsystem(dt);
    //m_LED = new LEDSubsystem(m_PhotonVision, dt);
    right5 = new Right5Ball(m_swerveSubsystem, m_intake, m_conveyor, m_shooter);
    mid4 = new Mid4Ball(m_swerveSubsystem, m_intake, m_conveyor, m_shooter);
    mid5 = new Mid5Ball(m_swerveSubsystem, m_intake, m_conveyor, m_shooter);
    left2 = new Left2BallD(m_swerveSubsystem, m_intake, m_conveyor, m_shooter);
    semicircle = new SemiCircle(m_swerveSubsystem);
    simple2 = new Simple2Ball(m_swerveSubsystem, m_intake, m_conveyor, m_shooter);
    fenderDelay = new FenderDelay(m_swerveSubsystem, m_intake, m_conveyor, m_shooter);
    straight = new Straight(m_swerveSubsystem);
    forward = new Forward(m_swerveSubsystem);

    m_swerveSubsystem.setDefaultCommand(new RunCommand(() -> dt.setModuleStates(m_scheme.getJoystickSpeeds()), m_swerveSubsystem));

    LiveWindow.disableAllTelemetry();

    m_PhotonVision.fieldSetup(m_swerveSubsystem.dt.getField());

    // Detect if controllers are missing / Stop multiple warnings
    DriverStation.silenceJoystickConnectionWarning(OI.PRACTICE);

    // Configure the button bindings
    configureButtonBindings();

    m_climb.setDefaultCommand(
      // Use left y axis to control the speed of the climber
      new RunCommand(
        () -> m_climb
          .setOutput(op.JoystickLY()), m_climb));

    
    autoChooser.setDefaultOption("Right5Ball", right5);
    autoChooser.addOption("Mid5Ball", mid5);
    autoChooser.addOption("Left2BallD", left2);
    autoChooser.addOption("Mid4Ball", mid4);
    autoChooser.addOption("Fender Delay", fenderDelay);
    autoChooser.addOption("CircleShot", simple2);
    autoChooser.addOption("SemiCircle", semicircle);
    autoChooser.addOption("Straight", straight);
    autoChooser.addOption("Forward", forward);
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
    drv.AButton.whileActiveOnce(new AutoAim(m_swerveSubsystem, m_PhotonVision, true, m_scheme));

    // While driver holds the B button Auto Aim to the closest ball using the left stick for distance control
    //drv.BButton.whileActiveOnce(new AutoAim(m_swerveSubsystem, m_PhotonVision, false, m_scheme));

    //drv.XButton.whileActiveOnce(new AutoAimRotate(m_swerveSubsystem, m_PhotonVision, true, m_scheme));

    //drv.YButton.whileActiveOnce(new AutoAimRotate(m_swerveSubsystem, m_PhotonVision, true, m_scheme));

    // When the left bumper is pressed on driver controller controls are slower
    drv.BumperL.whenActive(new ConditionalCommand(new InstantCommand(() -> {setDriveMaxSpeed(
        DRIVE.MAX_STRAFE_SPEED_SLOW, DRIVE.MAX_ROTATE_SPEED_SLOW);
        fast = false;}),
      new InstantCommand(() -> {setDriveMaxSpeed(DRIVE.MAX_STRAFE_SPEED_FAST, DRIVE.MAX_ROTATE_SPEED_FAST);
        fast = true;}),
      () -> fast));

    // Turn on the conveyor when the bottom sensor is blocked (ball waiting to go up)
    // unless top sensor blocked (the ball has no place to go)
    (topConveyorSensor.negate().and(frontConveyorSensor).and(op.BButton.negate()).and(auto))
    .whenActive(() -> {
          m_conveyor.m_ConveyorMotor.set(CONVEYOR.SPEED);
          drv.setRumble(0.8);
          op.setRumble(0.8);})
    .whenInactive(() -> {
          m_conveyor.m_ConveyorMotor.set(0);
          drv.setRumble(0);
          op.setRumble(0);});
  
    // When start button is pressed reorient the field drive to the current heading
    drv.StartButton.whenActive(() -> dt.zeroGyroscope());
    
    // Turn on the conveyor while held then off
    op.AButton.whenActive(() -> m_conveyor.on(CONVEYOR.SHOOTSPEED))
    .whenInactive(() -> m_conveyor.turnOff());

    // Spin the conveyor backwards when held
    op.BButton.whenActive(() -> {m_conveyor.turnBackwards(); m_intake.retractIntake(); m_intake.setOutput(-INTAKE.SPEED);})
        .whenInactive(() -> {m_conveyor.turnOff(); m_intake.setOutput(0);});

    // When right bumper is pressed raise/lower the intake and stop/start the intake on both controllers
    op.BumperR.whenActive(() -> m_intake.toggleIntake(true));

    op.StartButton.whenActive(() -> m_climb.toggleArms());

    // Spin up the shooter for the fender shot when the 'X' button is pressed.
    op.XButton.whenActive(new InstantCommand(() -> {
      m_shooter.setRPS(39.0, SHOOTER.FENDERFF);
      m_shooter.setHoodPosition(0);
    }, m_shooter));

    op.BackButton.whenActive(new InstantCommand(() -> {
      m_shooter.setRPS(39.0, SHOOTER.LAUNCHFF);
      m_shooter.setHoodPosition(SHOOTER.HOODCIRCLE);
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

  @Log
  public String getPose() {
    return dt.getPose().toString();
  }

  @Config
  private void setDriveMaxSpeed(double maxStrafe, double maxRotate) {
    m_swerveSubsystem.dt.setMaxSpeeds(maxStrafe, maxStrafe, maxRotate);
  }

  @Config
  private void setDriveMaxRamp(double maxStrafeRamp, double maxRotateRamp) {
    m_swerveSubsystem.dt.setMaxRamp(maxStrafeRamp, maxRotateRamp);
  }
}
