package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.TalonSRXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.INTAKE;
import frc.robot.UA6391.StallDetector;

public class IntakeSubsystem extends SubsystemBase implements Loggable{
    private final WPI_TalonSRX m_IntakeMotor = new WPI_TalonSRX(INTAKE.CANID);
    private final WPI_TalonSRX m_IntakeMotor2 = new WPI_TalonSRX(INTAKE.CANID2);
    private final DoubleSolenoid m_intakeSolenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, INTAKE.SOLENOID1FWD, INTAKE.SOLENOID1REV);
    
    TalonSRXSimCollection m_IntakeMotorSim = new TalonSRXSimCollection(m_IntakeMotor);

    //StallDetector intakeStall;
    // Since we are using DoubleSolenoids they must be initially set so when they are toggled they 
    // know which direction to toggle to as their default state is kOff.
    public IntakeSubsystem() {
        setOutput(0);
        m_intakeSolenoid1.set(Value.kForward);
        //intakeStall = new StallDetector(INTAKE.PDPSLOT);
        //intakeStall.setMinStallMillis(INTAKE.STALLTIME);
        m_IntakeMotor2.follow(m_IntakeMotor);
        m_IntakeMotor2.setInverted(InvertType.OpposeMaster);
        m_IntakeMotor.setStatusFramePeriod(1, 255);
        m_IntakeMotor.setStatusFramePeriod(2, 255);
        m_IntakeMotor2.setStatusFramePeriod(1, 255);
        m_IntakeMotor2.setStatusFramePeriod(2, 255);
    }

    @Config
    public void setOutput(double speed) {
        m_IntakeMotor.set(speed);
    }

    @Config
    public void extendIntake(boolean extend) {
        if (extend) {
            m_intakeSolenoid1.set(Value.kForward);
        }
        else {
            m_intakeSolenoid1.set(Value.kReverse);
        }
    }

    @Config
    public void toggleIntakePosition(boolean enabled) {
        m_intakeSolenoid1.toggle();       
    }

    @Config
    public void toggleIntake(boolean enabled) {
        // Only turn it on if intake is down and it is currently off
        if(m_IntakeMotor.get() == 0 && m_intakeSolenoid1.get() == DoubleSolenoid.Value.kReverse) {
            extendIntake(true);
            setOutput(INTAKE.SPEED);
        }
        else{
            retractIntake();
            setOutput(0);
        }
    }

/*     public void checkStall() {
        if (intakeStall.getStallStatus().isStalled) {
            setOutput(0);
        }
    } */

    public void deployIntake() {
        setOutput(INTAKE.SPEED);
        extendIntake(true);
    }

    public void retractIntake() {
        extendIntake(false);
        setOutput(0);
    }
}