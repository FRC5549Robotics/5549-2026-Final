package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
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

    private final PIDController hoodPID = new PIDController(0.08, 0.0, 0.004); // kP, kI, kD
    private static final double kS = 0.05;

    private double hoodSetpoint = 69.0;
    {
        hoodPID.setTolerance(0.5);
    }

    //private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final DutyCycleOut homingRequest = new DutyCycleOut(-0.2);

    TalonFX HoodMotor;
    TalonFXConfiguration HoodMotorConfig;

    public Hood(){
        HoodMotor = new TalonFX(Constants.HOOD_MOTOR_ID, "lil clanker");
        HoodMotorConfig = new TalonFXConfiguration();

        HoodMotorConfig.CurrentLimits.StatorCurrentLimit = 40; //used to be 60
        HoodMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        HoodMotorConfig.CurrentLimits.SupplyCurrentLimit = 40; //used to be 25
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
        pos = pos/5*(184.0/11.0) + 54; //switch to degrees, just trust, don't change
        return pos; //return in degrees
    }

    public void setAngle(double targetDeg) {
        targetDeg = MathUtil.clamp(targetDeg, 54, 78.5);
        hoodSetpoint = targetDeg;
    }

    public boolean atTarget() {
        //SmartDashboard.putBoolean("Hood at target", hoodPID.atSetpoint());
        return hoodPID.atSetpoint();
    }

    public void hoodDownSlow() {
        PIDEnabled = false;
        HoodMotor.setControl(homingRequest);
    }

    public void stop() {
        HoodMotor.setControl(new NeutralOut());
        System.out.println("hood motor stop() ran");
        PIDEnabled = true;
    }

    public void zeroEncoder() {
        HoodMotor.setPosition(0);
        System.out.println("hood zeroed");
    }

    public boolean atBottom() {
        boolean velocitySlow = Math.abs(HoodMotor.getVelocity().getValueAsDouble()) < 0.05;
        SmartDashboard.putBoolean("velocitySlow", velocitySlow);

        boolean currentSpiked = HoodMotor.getStatorCurrent().getValueAsDouble() > 40.0;
        SmartDashboard.putBoolean("currentSpiked", currentSpiked);

        return velocitySlow && currentSpiked;
    }

    @Override
    public void periodic() {
        double currentPos = getHoodPosition();

        SmartDashboard.putNumber("Hood Target", hoodSetpoint);
        SmartDashboard.putNumber("Hood Position", currentPos);
        SmartDashboard.putBoolean("Hood PID enabled", PIDEnabled);

        if (!PIDEnabled) return;

        double output = hoodPID.calculate(currentPos, hoodSetpoint);

        //SmartDashboard.putNumber("Hood Output", output);

        if (Math.abs(output) > 0.001) {
            output += Math.signum(output) * kS;
        }
        
        output = MathUtil.clamp(output, -0.25, 0.25);

        if (atTarget()) {
            HoodMotor.setControl(new NeutralOut());
        } else {
            HoodMotor.setControl(new DutyCycleOut(output).withEnableFOC(true)); //move the hood
        }
    }
}