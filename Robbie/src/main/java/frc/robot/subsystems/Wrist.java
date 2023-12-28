package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Wrist extends SubsystemBase{

    public enum ArmState{
        OFF,
        JOG,
        POSITION,
        ZERO
    }
    
    CANSparkMax armMaster = new CANSparkMax(ArmConstants.DEVICE_ID_ARM_MASTER, MotorType.kBrushless);
    CANSparkMax armSlave  = new CANSparkMax(ArmConstants.DEVICE_ID_ARM_SLAVE,  MotorType.kBrushless);

    ArmState state = ArmState.OFF;

    double jogValue = 0;
    Rotation2d setpoint = new Rotation2d();

    private static Wrist instance = new Wrist();

    public Wrist(){
        configMotors();
    }

    public static Wrist getInstance(){
        return instance;
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
                zero();
                break;
        }   
        
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(armMaster.getEncoder().getPosition() / ArmConstants.ARM_GEAR_RATIO);
    }

    public void zero(){
      if(!zero indicator)
        jog(0.1); //whatever speed is comfortable
      else{
        zeroEncoders();
        jog(0);
        setState(SubsystemState.STOW);
      }
    }

    public void set(double value){
        armMaster.set(value);
    }
    
    public void setJog(double jog){
        jogValue = jog;
    }

    private void goToSetpoint(){
        armMaster.getPIDController().setReference(setpoint.getRotations() * ArmConstants.ARM_GEAR_RATIO, ControlType.kSmartMotion);
    }

    public void setSetpoint(Rotation2d setpoint){
        setState(ArmState.POSITION);
        this.setpoint = setpoint;
    }

    public Rotation2d getSetpoint(){
        return setpoint;
    }

    public void setState(ArmState state){
        this.state = state;
    }

    public ArmState getState(){
        return state; 
    }

    public boolean atSetpoint(){
        return Math.abs(setpoint.minus(getAngle()).getRotations()) < ArmConstants.TOLERANCE.getRotations();
    }

    public void logData(){
        SmartDashboard.putNumber("Wrist Position", getAngle().getDegrees());
        SmartDashboard.putBoolean("Wrist At Setpoint", atSetpoint());
        SmartDashboard.putString("Wrist State", getState().toString());
        SmartDashboard.putNumber("Wrist Setpoint", getSetpoint().getDegrees());
    }


    public void configMotors(){
        armMaster.restoreFactoryDefaults();
        armSlave.restoreFactoryDefaults();

        armMaster.setInverted(false);
        armSlave.setInverted(armMaster.getInverted());

        armMaster.setIdleMode(IdleMode.kBrake);
        armSlave.setIdleMode(armMaster.getIdleMode());

        armMaster.setSmartCurrentLimit(40, 40);
        armSlave.setSmartCurrentLimit(40, 40);

        SparkMaxPIDController armController = armMaster.getPIDController();
        armController.setP(ArmConstants.ARM_kP);
        armController.setD(ArmConstants.ARM_kD);

        armController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        armController.setSmartMotionMaxAccel(ArmConstants.MAX_ACCELERATION, 0);
        armController.setSmartMotionMaxVelocity(ArmConstants.MAX_VELOCITY, 0);

        armSlave.follow(armMaster);
    }
}
