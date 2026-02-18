package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class Hood extends SubsystemBase{

    private boolean PIDEnabled = false; //default to PID being off

    private final PIDController hoodPID = new PIDController(0.25, 0.0, 0.001); // kP, kI, kD
    private double hoodSetpoint = 72.0;
    {
        hoodPID.setTolerance(0.4);
    }

    TalonFX HoodMotor;
    TalonFXConfiguration HoodMotorConfig;

    public Hood(){
        HoodMotor = new TalonFX(Constants.HOOD_MOTOR_ID, "lil clanker");
        HoodMotorConfig = new TalonFXConfiguration();
        HoodMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        HoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        HoodMotor.getConfigurator().apply(HoodMotorConfig);
    }

    public double getHoodPosition() {
        double pos = HoodMotor.getPosition().getValueAsDouble();
        pos = pos/5*(184/11) + 22.3 + 45; //switch to degrees, just trust, don't change
        return pos;
    }

    public void HoodUp(){

        double pos = getHoodPosition();

        if (pos < 78) {
            HoodMotor.set(0.2);  // move up
            System.out.println("Hood going up, Hood go vroom vroom");
        } else {
            HoodMotor.set(0.0);  // stop at the limit
            System.out.println("Hood at upper limit!");
        }

        SmartDashboard.putNumber("Hood Position", pos);
    }

    public void HoodDown() {

        double pos = getHoodPosition();

        if (pos > 69) {   // assuming 68 is your bottom
            HoodMotor.set(-0.2);
            System.out.println("Hood going down, Twinkle twinkle little star");
        } else {
            HoodMotor.set(0.0);
            System.out.println("Hood at lower limit!");
        }

        SmartDashboard.putNumber("Hood Position", pos);

    }

    public void setAngle(double targetPos) {
        targetPos = MathUtil.clamp(targetPos, 69.0, 78.0);
        hoodSetpoint = targetPos;
    }
    
    public void HoodOff() {
        HoodMotor.set(0.0);
    }

    public void hoodDownSlow() {
        HoodMotor.set(-0.25);
    }

    public void stop() {
        HoodMotor.set(0.0);
    }

    public void zeroEncoder() {
        HoodMotor.setPosition(0);
        PIDEnabled = true; //allow PID to move the hood
    }

    public boolean atBottom() {
        return Math.abs(HoodMotor.getVelocity().getValueAsDouble()) < 0.05;
    }

    @Override
    public void periodic() {
        double currentPos = getHoodPosition();

        double output = hoodPID.calculate(currentPos, hoodSetpoint);
        
        output = MathUtil.clamp(output, -0.25, 0.25);

        if (Math.abs(hoodSetpoint - currentPos) < 0.65) {
            output = 0.0;
        }

        if (PIDEnabled == true) { //if the hood has been zeroed...
            HoodMotor.set(output); //move the hood
        }

        SmartDashboard.putNumber("Hood Target", hoodSetpoint);
        SmartDashboard.putNumber("Hood Position", currentPos);
        SmartDashboard.putNumber("Hood Output", output);
    }
}