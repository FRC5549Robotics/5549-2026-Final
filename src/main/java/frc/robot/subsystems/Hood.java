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

    //Create PID. kP is how fast the motor tries to close the error. kD is for adjusting speed based on how
    //quickly the error is decreasing. kI isn't necessary
    private final PIDController hoodPID = new PIDController(0.15, 0.0, 0.001);

    private double hoodTarget = 69; //create hoodTarget
    private boolean PIDEnabled = false; //default to PID being off

    {
        hoodPID.setTolerance(0.7); //Within what range should the PID stop
    }

    TalonFX HoodMotor; // create HoodMotor
    TalonFXConfiguration HoodMotorConfig;

    public Hood(){
        HoodMotor = new TalonFX(Constants.HOOD_MOTOR_ID, "lil clanker"); //further define hoodMotor
        HoodMotorConfig = new TalonFXConfiguration();
        HoodMotorConfig.CurrentLimits.StatorCurrentLimit = 60; //current limit
        HoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; //default mode is coast mode

        HoodMotor.getConfigurator().apply(HoodMotorConfig);
    }

    public double getHoodPosition() {
        double pos = HoodMotor.getPosition().getValueAsDouble(); // take hoodMotors position in rotations
        pos = pos/5*(184/11) + 67.3; //switch to degrees, just trust, don't change
        return pos;
    }

    public void setAngle(double targetPos) {
        System.out.println(targetPos);
        targetPos = MathUtil.clamp(targetPos, 69.0, 78.0); //Range angles can be between
        hoodTarget = targetPos; //update hoodTarget to the new target
    }

    public void hoodDownInitial() {
        HoodMotor.set(-0.25); //move hood down
    }

    public void stop() {
        System.out.println("stop moving the hood");
        HoodMotor.set(0.0); //stop moving hood
    }

    public void zeroEncoder() {
        HoodMotor.setPosition(0); //zero encoder for consistency
        PIDEnabled = true; //allow PID to move the hood
    }

    public boolean atBottom() {
        return Math.abs(HoodMotor.getVelocity().getValueAsDouble()) < 0.05; //if the motor slows down enough, return that it's hit the hard stop
    }

    @Override
    public void periodic() {
        double currentPos = getHoodPosition(); //get hood's position

        double output = hoodPID.calculate(currentPos, hoodTarget); //calculate how to run motor based on PID
        
        output = MathUtil.clamp(output, -0.25, 0.25); //don't run motor too fast

        if (hoodPID.atSetpoint()) {
            output = 0.0; //If hood is close enough, don't run motor
        }

        if (PIDEnabled == true) { //if the hood has been zeroed...
            //System.out.println(output);
            HoodMotor.set(output); //move the hood
        }

        SmartDashboard.putNumber("Hood Target", hoodTarget);
        SmartDashboard.putNumber("Hood Position", currentPos);
        SmartDashboard.putNumber("Hood Output", output);
    }
}