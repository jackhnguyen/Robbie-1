package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawConstants;

public class Claw extends SubsystemBase{
    
    DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ClawConstants.DEVICE_ID_FORWARD, ClawConstants.DEVICE_ID_REVERSE);

    private static Claw instance = new Claw();

    public static Claw getInstance(){
        return instance;
    }

    public void set(Value value){
        solenoid.set(value);
    }

    public void toggleClaw(){
        if(solenoid.get() == Value.kForward){
            set(Value.kReverse);
        }
        else
            set(Value.kForward);
    }

    public void close(){
        set(Value.kForward);
    }

    public void open(){
        set(Value.kReverse);
    }

    public boolean isClosed(){
        return solenoid.get() == Value.kForward;
    }

    public Value getValue(){
        return solenoid.get();
    }
}
