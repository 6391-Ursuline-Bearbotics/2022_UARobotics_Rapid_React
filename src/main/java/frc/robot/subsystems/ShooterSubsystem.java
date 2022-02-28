package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import java.time.Period;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SHOOTER;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
  @Log
  // private final WPI_TalonSRX m_shooterMotor = new WPI_TalonSRX(SHOOTER.kShooterMotorPort2);
  private final WPI_TalonFX m_shooterMotor = new WPI_TalonFX(SHOOTER.MOTORPORT);

  // private final WPI_TalonSRX m_shooterMotor2 = new WPI_TalonSRX(SHOOTER.kShooterMotorPort);
  private final WPI_TalonFX m_shooterMotor2 = new WPI_TalonFX(SHOOTER.MOTOR2PORT);

  private final WPI_TalonSRX m_hood = new WPI_TalonSRX(SHOOTER.HOODPORT);

  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(SHOOTER.kSVOLTS,
      SHOOTER.kVVOLTSECONDSPERROTATION, SHOOTER.kA);

  private final BangBangController m_bangBangController = new BangBangController(SHOOTER.TOLERANCERPS);

  private double rpsAdjustment = 0.0;

  // The shooter subsystem for the robot.
  public ShooterSubsystem() {
    TalonFXConfiguration flywheelTalonConfig = new TalonFXConfiguration();
    flywheelTalonConfig.peakOutputForward = 0.5; // Only for initial testing
    flywheelTalonConfig.peakOutputReverse = 0;
    flywheelTalonConfig.slot0.kP = SHOOTER.P;
    flywheelTalonConfig.slot0.kD = SHOOTER.D;
    flywheelTalonConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_1Ms;
    flywheelTalonConfig.velocityMeasurementWindow = 1;
    m_shooterMotor.configAllSettings(flywheelTalonConfig);

    m_shooterMotor.configVoltageCompSaturation(12);
    m_shooterMotor.enableVoltageCompensation(true);
    m_shooterMotor2.configVoltageCompSaturation(12);
    m_shooterMotor2.enableVoltageCompensation(true);

    m_shooterMotor2.follow(m_shooterMotor);
    m_shooterMotor.setInverted(false);
    m_shooterMotor2.setInverted(InvertType.OpposeMaster);
    m_shooterMotor.setNeutralMode(NeutralMode.Coast);
		m_shooterMotor2.setNeutralMode(NeutralMode.Coast);

    m_hood.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    m_hood.config_kP(0, 0.0001);
    m_hood.setSensorPhase(false);
  }

  @Log(name = "Current Shooter Speed (RPS)")
  public double getShooterSpeed() {
    return m_shooterMotor.getSelectedSensorVelocity() * SHOOTER.RAWTOFLYWHEELRPS;
  }

  @Log
  public double getShooterVoltage() {
    return m_shooterMotor.getBusVoltage();
  }

  @Log
  @Log(tabName = "Dashboard", name = "Shooter Setpoint")
  public double getSetpoint() {
    if (m_shooterMotor.getControlMode() == ControlMode.Velocity) {
      return m_shooterMotor.getClosedLoopTarget() * SHOOTER.RAWTOFLYWHEELRPS;
    }
    else {
      return 0.0;
    }
    
  }

  @Log
  @Log(tabName = "Dashboard", name = "Good to Shoot?")
  public boolean atSetpoint() {
    return m_bangBangController.atSetpoint() && getSetpoint() > 0;
  }

  @Config
  public void setRPS(double rps) {
    var ff = m_shooterFeedforward.calculate(rps);
    m_shooterMotor.set(ControlMode.Velocity, rps * SHOOTER.RPSTORAW, DemandType.ArbitraryFeedForward, ff);
  }

  // @Config
  // public void setVoltage(double voltage) {
  //   m_shooterMotor.setVoltage(voltage);
  // }

  @Config
  public void setHoodPosition(double encoderPosition) {
    m_hood.set(ControlMode.Position, encoderPosition);
  }

  @Log
  public double getHoodPosition() {
    return m_hood.getSelectedSensorPosition();
  }

/*   @Log
  public boolean getBallShot() {
    return m_shooterMotor.getSupplyCurrent() //maybe should just use top prox?
  } */
}