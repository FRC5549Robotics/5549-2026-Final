package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;

public class Hood extends SubsystemBase{

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final TorqueCurrentFOC homingRequest = new TorqueCurrentFOC(-25);

    private double hoodTarget;

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

        HoodMotorConfig.Slot0.kP = 20;
        HoodMotorConfig.Slot0.kI = 0;
        HoodMotorConfig.Slot0.kD = 0.2;
        HoodMotorConfig.Slot0.kG = 0.35; //some gravity PID thing
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
        return pos;
    }

    public void setAngle(double targetDeg) {
        targetDeg = MathUtil.clamp(targetDeg, 69.0, 78.0);
        hoodTarget = targetDeg;

        double motorRot = degreesToMotorRot(targetDeg);

        HoodMotor.setControl(positionRequest.withPosition(motorRot));
    }

    public boolean atTarget() {
        double error = HoodMotor.getClosedLoopError().getValueAsDouble();
        return Math.abs(error) < 1;
    }

    public void hoodDownSlow() {
        HoodMotor.setControl(homingRequest);
    }

    public void stop() {
        HoodMotor.setControl(positionRequest.withPosition(HoodMotor.getPosition().getValueAsDouble()));
        System.out.println("hood motor stop() ran");
    }

    public void zeroEncoder() {
        HoodMotor.setPosition(0);
    }

    public boolean atBottom() {
        System.out.println(HoodMotor.getVelocity().getValueAsDouble());
        return Math.abs(HoodMotor.getVelocity().getValueAsDouble()) < 0.05; //if the motor slows down enough, return that it's hit the hard stop
    }

    @Override
    public void periodic() {
        double currentPos = getHoodPosition();

        if (atTarget()) {
            HoodMotor.setControl(new NeutralOut());
        }

        SmartDashboard.putNumber("Hood Target", hoodTarget);
        SmartDashboard.putNumber("Hood Position", currentPos);
    }
}