package frc.robot.subsystems;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.motorcontrol.InvertType;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import frc.robot.Constants.SHOOTER;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
  @Log
  // private final WPI_TalonSRX m_shooterMotor = new WPI_TalonSRX(SHOOTER.kShooterMotorPort2);
  private final WPI_TalonFX m_shooterMotor = new WPI_TalonFX(SHOOTER.kShooterMotorPort2);

  // private final WPI_TalonSRX m_shooterMotor2 = new WPI_TalonSRX(SHOOTER.kShooterMotorPort);
  private final WPI_TalonFX m_shooterMotor2 = new WPI_TalonFX(SHOOTER.kShooterMotorPort);

  private final SimpleMotorFeedforward m_shooterFeedforward = new SimpleMotorFeedforward(SHOOTER.kSVolts,
      SHOOTER.kVVoltSecondsPerRotation, SHOOTER.kA);

  private final BangBangController m_bangBangController = new BangBangController(SHOOTER.ToleranceRPS);

  @Log
  private double m_desiredVelocityRPS;

  // The Kv and Ka constants are found using the FRC Characterization toolsuite.
  LinearSystem<N2, N1, N1> m_flywheelPosition = LinearSystemId.identifyPositionSystem(SHOOTER.kVVoltSecondsPerRotation, SHOOTER.kA);
  LinearSystemSim<N2, N1, N1> m_flywheelPositionSim = new LinearSystemSim<>(m_flywheelPosition);

  //private final DCMotor m_flywheelGearbox = DCMotor.getVex775Pro(2);
  //private final FlywheelSim m_flywheelSim = new FlywheelSim(m_flywheelPositionSim, m_flywheelGearbox, 2);

  // The shooter subsystem for the robot.
  public ShooterSubsystem() {
    TalonFXConfiguration flywheelTalonConfig = new TalonFXConfiguration();
    flywheelTalonConfig.slot0.kP = SHOOTER.P;
    flywheelTalonConfig.slot0.kD = SHOOTER.D;
    m_shooterMotor.configAllSettings(flywheelTalonConfig);

    m_shooterMotor2.follow(m_shooterMotor);
    m_shooterMotor.setInverted(false);
    m_shooterMotor2.setInverted(InvertType.OpposeMaster);
    m_shooterMotor.setNeutralMode(NeutralMode.Coast);
		m_shooterMotor2.setNeutralMode(NeutralMode.Coast);
  }

  @Override
  public void periodic() {
    double voltage = m_shooterFeedforward.calculate(m_desiredVelocityRPS)
     + m_bangBangController.calculate(getTalonVelocity(), m_desiredVelocityRPS) * kNominalVoltage;
    m_shooterMotor.setVoltage(voltage);
    
    SmartDashboard.putNumber("ff applied voltage", voltage);
    SmartDashboard.putNumber("talon applied voltage", m_shooterMotor.getBusVoltage());
  }

  @Override
  public void simulationPeriodic() {
    // In this method, we update our simulation of what our elevator is doing
    // First, we set our "inputs" (voltages)
    m_flywheelPositionSim.setInput(m_shooterMotor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_flywheelPositionSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_flywheelPositionSim.getOutput(0));
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_flywheelPositionSim.getCurrentDrawAmps()));
  }

  @Log
  @Log(tabName = "Dashboard", name = "Shooter Speed")
  public double getMeasurement() {
    // old way was getRate
    m_angle = getAngle();
    m_time = Timer.getFPGATimestamp();
    
    m_angularVelocity = m_velocityFilterMA.calculate((m_angle - m_lastAngle) / (m_time - m_lastTime));
    m_lastTime = m_time;
    m_lastAngle = m_angle;

    m_angularVelocity = m_velocityFilterIIR.calculate(m_angularVelocity);
    return m_angularVelocity;
  }

  @Log
  public double getShooterVoltage() {
    return m_shooterMotor.get();
  }

  @Log
  @Log(tabName = "Dashboard", name = "Shooter Setpoint")
  public double getSetpoint() {
    return shooterPID.getSetpoint();
  }

  @Log
  @Log(tabName = "Dashboard", name = "Good to Shoot?")
  public boolean atSetpoint() {
    return shooterPID.atSetpoint() && shooterPID.getSetpoint() > 0 && this.isEnabled();
  }

  @Config
  public void setShooterRPS(double rps) {
    this.setSetpoint(rps);
  }

  @Config
  public void setVoltage(double voltage) {
    m_shooterMotor.setVoltage(voltage);
  }
}