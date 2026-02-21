package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class Hood extends SubsystemBase{

    private boolean PIDEnabled = false; //default to PID being off

    private final PIDController hoodPID = new PIDController(0.15, 0.0, 0.001); // kP, kI, kD
    private double hoodSetpoint = 72.0;
    {
        hoodPID.setTolerance(0.5);
    }

    //private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final TorqueCurrentFOC homingRequest = new TorqueCurrentFOC(-25);

    TalonFX HoodMotor;
    TalonFXConfiguration HoodMotorConfig;

    public Hood(){
        HoodMotor = new TalonFX(Constants.HOOD_MOTOR_ID, "lil clanker");
        HoodMotorConfig = new TalonFXConfiguration();

        HoodMotorConfig.CurrentLimits.StatorCurrentLimit = 60;
        HoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodMotorConfig.CurrentLimits.SupplyCurrentLimit = 25;
        HoodMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        HoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        HoodMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        HoodMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

        HoodMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 4; // cap on velocity
        HoodMotorConfig.MotionMagic.MotionMagicAcceleration = 8; // cap on acceleration

        HoodMotor.getConfigurator().apply(HoodMotorConfig);
    }

    private double degreesToMotorRot(double deg) {
        return (deg-67.3) / (184.0/11.0) * 5;
    }

    public double getHoodPosition() {
        double pos = HoodMotor.getPosition().getValueAsDouble();
        pos = pos/5*(184.0/11.0) + 67.35; //switch to degrees, just trust, don't change
        return pos; //return in degrees
    }

    public void setAngle(double targetDeg) {
        targetDeg = MathUtil.clamp(targetDeg, 69.0, 78.0);
        hoodSetpoint = targetDeg;
    }

    public boolean atTarget() {
        double error = hoodSetpoint - getHoodPosition();
        System.out.println(error);
        return Math.abs(error) < 0.4;
    }

    public void hoodDownSlow() {
        HoodMotor.setControl(homingRequest);
    }

    public void stop() {
        HoodMotor.setControl(new NeutralOut());
        System.out.println("hood motor stop() ran");
    }

    public void zeroEncoder() {
        HoodMotor.setPosition(0);
        PIDEnabled = true;
    }

    public boolean atBottom() {
        return Math.abs(HoodMotor.getVelocity().getValueAsDouble()) < 0.05; //if the motor slows down enough, return that it's hit the hard stop
    }

    @Override
    public void periodic() {
        double currentPos = getHoodPosition();

        double output = hoodPID.calculate(currentPos, hoodSetpoint);
        
        output = MathUtil.clamp(output, -0.25, 0.25);

        if (PIDEnabled == true) { //if the hood has been zeroed...
            HoodMotor.setControl(new DutyCycleOut(output).withEnableFOC(true)); //move the hood
        }

        SmartDashboard.putNumber("Hood Target", hoodSetpoint);
        SmartDashboard.putNumber("Hood Position", currentPos);
        SmartDashboard.putNumber("Hood Output", output);
    }
}