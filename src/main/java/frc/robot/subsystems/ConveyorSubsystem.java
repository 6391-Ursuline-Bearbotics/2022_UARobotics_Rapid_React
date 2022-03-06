package frc.robot.subsystems;

import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PowerDistribution;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Constants.CONVEYOR;

public class ConveyorSubsystem extends SubsystemBase implements Loggable{
    private final WPI_TalonSRX m_ConveyorMotor = new WPI_TalonSRX(CONVEYOR.CANID);

    //PowerDistribution m_PDP = new PowerDistribution();

    DigitalInput frontconveyor = new DigitalInput(CONVEYOR.FRONTSENSORPORT);
    DigitalInput topconveyor = new DigitalInput(CONVEYOR.TOPSENSORPORT);

    // The averaging here is so that supurious noise spikes don't trip the sensors.
    public ConveyorSubsystem() {}

    @Config
    public void turnOff() {
        this.m_ConveyorMotor.set(0);
    }

    @Config
    public void on(double speed) {
        this.m_ConveyorMotor.set(speed);
    }

    @Config
    public void turnBackwards() {
        this.m_ConveyorMotor.set(CONVEYOR.BACKSPEED);
    }

    @Config.ToggleButton
    public void toggleConveyor(boolean enabled) {
        if(this.m_ConveyorMotor.get() > 0) {
            turnOff();
        }
        else{
            on(CONVEYOR.SPEED);
        }   
    }

    @Log
    @Log(tabName = "Dashboard", name = "Front Sensor")
    public boolean getFrontConveyor() {
        return !frontconveyor.get();
    }

    @Log
    @Log(tabName = "Dashboard", name = "Top Sensor")
    public boolean getTopConveyor() {
        return !topconveyor.get();
    }
}