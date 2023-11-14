package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ExtenderConstants;

public class Extender extends SubsystemBase{

    public enum ExtenderState{
        OFF,
        JOG,
        POSITION,
        ZERO
    }
    
    private static Extender instance = new Extender();

    ExtenderState state = ExtenderState.OFF;
    
    double jogValue = 0;
    double setpoint = 0;

    CANSparkMax extenderMotor = new CANSparkMax(ExtenderConstants.DEVICE_ID_EXTENDER, MotorType.kBrushless);

    public static Extender getInstance(){
        return instance;
    }

    public Extender(){
        configMotor();
    }

    @Override
    public void periodic() {
        logData();
        
        switch(state){
            case OFF:
                set(0);
                break;
            case JOG:
                set(jogValue);
                break;
            case POSITION:
                goToSetpoint();
                break;
            case ZERO:
                break;

        }
    }

    public double getPosition(){
        return extenderMotor.getEncoder().getPosition();
    }

    public void set(double value){
        extenderMotor.set(value);
    }

    private void goToSetpoint(){
        extenderMotor.getPIDController().setReference(setpoint, ControlType.kSmartMotion);
    }

    public void setSetpoint(double setpoint){
        setState(ExtenderState.POSITION);
        this.setpoint = setpoint;
    }

    public double getSetpoint(){
        return setpoint;
    }

    public ExtenderState getState(){
        return state;
    }

    public void setState(ExtenderState state){
        this.state = state;
    }

    public boolean atSetpoint(){
        return Math.abs(setpoint - getPosition()) < ExtenderConstants.TOLERANCE;
    }

    public void logData(){
        SmartDashboard.putNumber("Extender Position", getPosition());
        SmartDashboard.putBoolean("Extender At Setpoint", atSetpoint());
        SmartDashboard.putString("Extender State", getState().toString());
        SmartDashboard.putNumber("Extender Setpoint", getSetpoint());
    }

    public void configMotor(){
        extenderMotor.restoreFactoryDefaults();

        extenderMotor.setInverted(false);
        extenderMotor.setIdleMode(IdleMode.kBrake);
        extenderMotor.setSmartCurrentLimit(30, 30);

        SparkMaxPIDController controller = extenderMotor.getPIDController();

        controller.setP(ExtenderConstants.EXTENDER_kP);
        controller.setD(ExtenderConstants.EXTENDER_kD);

        controller.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        controller.setSmartMotionMaxAccel(ExtenderConstants.MAX_ACCELERATION, 0);
        controller.setSmartMotionMaxVelocity(ExtenderConstants.MAX_VELOCITY, 0);
    }
}

