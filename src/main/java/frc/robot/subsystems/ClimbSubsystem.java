package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CLIMB;
import edu.wpi.first.networktables.NetworkTableInstance;

public class ClimbSubsystem extends SubsystemBase implements Loggable{
    private WPI_TalonFX m_climbMotor;

    @Log
    private int climbinvert = 1;

    @Log
    public int climbstage = 0;

    @Log
    public int setpoint = 4200;

    public ClimbSubsystem(WPI_TalonFX m_climbMotor) {
        this.m_climbMotor = m_climbMotor;

        setOutput(0);
        m_climbMotor.setInverted(false);
        m_climbMotor.setSensorPhase(true);
        m_climbMotor.config_kP(0, CLIMB.P);
    }

    public static ClimbSubsystem Create() {
        WPI_TalonFX m_climbMotor = new WPI_TalonFX(CLIMB.CANID);
        return new ClimbSubsystem(m_climbMotor);
    }

    // This is the open loop control of the climber when the "triggers" are used to manually control it
    public void setOutput(double rightMotorPercent) {
        this.m_climbMotor.set(rightMotorPercent * climbinvert);
    }

    // This is the closed loop position control of the climber
    @Config
    public void setPosition(double position) {
        m_climbMotor.set(ControlMode.Position, position);
    }

    @Log
    public double getRightPosition() {
        return m_climbMotor.getSelectedSensorPosition();
    } 

    @Config.ToggleButton
    public void resetEnc(boolean enabled) {
        m_climbMotor.setSelectedSensorPosition(0);
        climbstage = 0;
    }

    @Config
    public void chirp(double freq) {
        m_climbMotor.set(ControlMode.MusicTone, freq);
    }

    // There are 3 "stages" that we go to.  While lining up we go to the first stage which raise
    // the climbers all the way up.  Once we move forward and touch the bar we bring them down
    // to the point they are on the bar but not actually pulling allowing teammates to get on but
    // it they were to pull ahead of time we would raise too.  Then the actual hang.
/*     @Config.ToggleButton
    public void nextClimbStage(boolean enabled) {
        climbstage = climbstage + 1;
        switch(climbstage) {
            case 1:
                setpoint = ClimbConstants.kFullUpEncoderCount;
                setPosition(ClimbConstants.kFullUpEncoderCount);
                break;
            case 2:
                setpoint = ClimbConstants.kOnBarEncoderCount;
                setPosition(ClimbConstants.kOnBarEncoderCount);
                break;
            case 3:
                setpoint = ClimbConstants.kHangingEncoderCount;
                setPosition(ClimbConstants.kHangingEncoderCount);
                break;
            default:
        }
    } */

    // Determines if the talon is at the desired position
    @Log
    public boolean atposition() {
        return inRange(m_climbMotor.getSelectedSensorPosition(), setpoint)
        && m_climbMotor.getSelectedSensorPosition() > 100;
    }

    public Boolean inRange(double position, double setpoint){
        if(position > setpoint + CLIMB.TOLERANCE){
            return false;
        }else if(position < setpoint - CLIMB.TOLERANCE){
            return false;
        }else{
            return true;
        }
    }
}