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
import frc.robot.UA6391.StatusFrameHelper;

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
        raiseIntake();
        //intakeStall = new StallDetector(INTAKE.PDPSLOT);
        //intakeStall.setMinStallMillis(INTAKE.STALLTIME);
        m_IntakeMotor2.follow(m_IntakeMotor);
        m_IntakeMotor2.setInverted(InvertType.OpposeMaster);
        StatusFrameHelper.statusFrameOffExcept1(m_IntakeMotor);
        StatusFrameHelper.statusFrameOff(m_IntakeMotor2);
    }

    @Config
    public void setOutput(double speed) {
        m_IntakeMotor.set(speed);
    }

    @Config
    public void lowerIntake() {
        m_intakeSolenoid1.set(Value.kForward);
    }

    @Config
    public void raiseIntake() {
        m_intakeSolenoid1.set(Value.kReverse);
    }

    @Config
    public void toggleIntakePosition(boolean enabled) {
        m_intakeSolenoid1.toggle();       
    }

    @Config
    public void toggleIntake(boolean enabled) {
        // Only turn it on if intake is down and it is currently off
        if(m_IntakeMotor.get() == 0 && m_intakeSolenoid1.get() == DoubleSolenoid.Value.kReverse) {
            deployIntake();
        }
        else{
            retractIntake();
        }
    }

    public void deployIntake() {
        setOutput(INTAKE.SPEED);
        lowerIntake();
    }

    public void retractIntake() {
        raiseIntake();
        setOutput(0);
    }
}