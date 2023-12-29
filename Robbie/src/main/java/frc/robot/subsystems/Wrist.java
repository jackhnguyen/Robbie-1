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

    private static Wrist instance = new Wrist(); //global instance; relieves from manually referencing wrist in other classes
    //when calling method from wrist fom another class the format is className.Method() bc wrist instantiazation is static
    public Wrist(){
        configMotors(); //sets up motor and stuff
    }

    public static Wrist getInstance(){  //gets class instance
        return instance;
    }

    @Override
    public void periodic() { //periodic = constantly running
        logData(); //logs data using smartdashboard
        
        switch(state){  //forever if else loop
            case OFF:          //if state = OFF than the loop stops and it runs the method set() 
                set(0);
                break; 
            case JOG: //motor power to limit overshoot n stuff
                set(jogValue);      //if state = JOG than loop stops and it runs set()
                break;
            case POSITION: //gets position of wrist ex; it's angle 
                goToSetpoint();
                break;
            case ZERO:  //wrist turned off 
                zero();
                break;
        }   
        
    }

    public Rotation2d getAngle(){ //converts motor turns into angle form
        return Rotation2d.fromRotations(armMaster.getEncoder().getPosition() / ArmConstants.ARM_GEAR_RATIO);
    }

    public void zero(){ //if not zero than jog will turn motor at 0.1 power, if zero than motor stops
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
        armMaster.restoreFactoryDefaults();  //default motar controlr settyn
        armSlave.restoreFactoryDefaults();   

        armMaster.setInverted(false);   //DOES NOT INVERT
        armSlave.setInverted(!armMaster.getInverted()); //copies opposite of master value

        armMaster.setIdleMode(IdleMode.kBrake);   //sets motor setting 
        armSlave.setIdleMode(armMaster.getIdleMode()); //slave copies master

        armMaster.setSmartCurrentLimit(40, 40); //freelimit: max amps into motor
        armSlave.setSmartCurrentLimit(40, 40); //stalllimit: enough amps to resist torque
        SparkMaxPIDController armController = armMaster.getPIDController(); //create motor with its pid controller
        armController.setP(ArmConstants.ARM_kP); //pid
        armController.setD(ArmConstants.ARM_kD); //pid 

        armController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        armController.setSmartMotionMaxAccel(ArmConstants.MAX_ACCELERATION, 0);
        armController.setSmartMotionMaxVelocity(ArmConstants.MAX_VELOCITY, 0);

        armSlave.follow(armMaster); //slave copies values of master
    }
}
