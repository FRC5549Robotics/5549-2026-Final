package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Hood extends SubsystemBase{

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

        double currentPos = getHoodPosition();

        SmartDashboard.putNumber("Current Pos", currentPos);

        double error = targetPos - currentPos;

        // ---- Tuning constants ----
        double kP = 0.23; // how aggressive it moves
        double maxSpeed = 0.25; // clamp so it doesn’t slam
        double minSpeed = 0.05; // helps overcome friction
        double tolerance = .6; // how close is "good enough"

        // If we're close enough, stop
        if (Math.abs(error) < tolerance) {
            HoodMotor.set(0.0);
            return;
        }

        // Proportional control
        double speed = error * kP;

        // Clamp speed so it behaves nicely
        if (speed > maxSpeed) speed = maxSpeed;
        if (speed < -maxSpeed) speed = -maxSpeed;

        // Make sure we still move if very close
        if (speed > 0 && speed < minSpeed) speed = minSpeed;
        if (speed < 0 && speed > -minSpeed) speed = -minSpeed;

        HoodMotor.set(speed);

        SmartDashboard.putNumber("Hood Target", targetPos);
        SmartDashboard.putNumber("Hood Error", error);
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
    }

    public boolean atBottom() {
        return Math.abs(HoodMotor.getVelocity().getValueAsDouble()) < 0.05;
    }

}


