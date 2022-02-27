/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private boolean userButton = false;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // The first argument is the root container
    // The second argument is whether logging and config should be given separate tabs
    Logger.configureLoggingAndConfig(m_robotContainer, false);
  }

  @Override
  public void simulationPeriodic() {
    // Here we calculate the battery voltage based on drawn current.
    // As our robot draws more power from the battery its voltage drops.
    // The estimated voltage is highly dependent on the battery's internal
    // resistance.
    /* double drawCurrent = m_robotContainer.getRobotDrive().getDrawnCurrentAmps();
    double loadedVoltage = BatterySim.calculateDefaultBatteryLoadedVoltage(drawCurrent);
    RoboRioSim.setVInVoltage(loadedVoltage);
    m_robotContainer.m_PhotonVision.visionSys.processFrame(m_robotContainer.m_robotDrive.getCurrentPose()); */
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    Logger.updateEntries();

    if (RobotController.getUserButton() && !userButton) {
      m_robotContainer.m_PhotonVision.m_HD3000.takeInputSnapshot();
      m_robotContainer.m_PhotonVision.m_HD3000.takeOutputSnapshot();
      m_robotContainer.m_PhotonVision.m_limelight.takeInputSnapshot();
      m_robotContainer.m_PhotonVision.m_limelight.takeOutputSnapshot();
    }
    userButton = RobotController.getUserButton();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
    Shuffleboard.selectTab("Dashboard");
    m_robotContainer.m_PhotonVision.lightsOff();
  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.m_PhotonVision.lightsOn();
    if (m_robotContainer.dt.getGyroReady()) {
      m_robotContainer.m_LED.rainbow();
    }
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.m_intake.setOutput(0);
    //m_robotContainer.m_intake.extendIntake(false);
/*     m_robotContainer.m_conveyor.turnOff();
    m_robotContainer.m_climb.invertclimber(false);
    m_robotContainer.m_climb.setOutput(0, 0);
    m_robotContainer.m_climb.climbstage = 0;
    m_robotContainer.m_climb.resetEnc(true);
    m_robotContainer.m_shooter.setRPS(0); */
    m_robotContainer.m_PhotonVision.lightsOff();

    // schedule the autonomous command (example)
    //move this down
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_intake.setOutput(0);
    //m_robotContainer.m_intake.extendIntake(false);
/*     m_robotContainer.m_conveyor.turnOff();
    m_robotContainer.m_climb.invertclimber(false);
    m_robotContainer.m_climb.setOutput(0, 0);
    m_robotContainer.m_climb.climbstage = 0;
    m_robotContainer.m_climb.resetEnc(true);
    m_robotContainer.m_shooter.setRPS(0); */
    m_robotContainer.m_PhotonVision.lightsOff();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
