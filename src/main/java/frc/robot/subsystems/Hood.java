package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;

public class Hood extends SubsystemBase{

    private final PositionVoltage positionRequest = new PositionVoltage(0).withEnableFOC(true);

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

        HoodMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        HoodMotorConfig.Slot0.kP = 20;
        HoodMotorConfig.Slot0.kI = 0;
        HoodMotorConfig.Slot0.kD = 0.2;

        HoodMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

        HoodMotor.getConfigurator().apply(HoodMotorConfig);
    }

    private double degreesToMotorRot(double deg) {
        return (deg-67.3) / (184.0/11.0) * 5;
    }

    public double getHoodPosition() {
        double pos = HoodMotor.getPosition().getValueAsDouble();
        pos = pos/5*(184/11) + 22.3 + 45; //switch to degrees, just trust, don't change
        return pos;
    }

    public void setAngle(double targetDeg) {
        targetDeg = MathUtil.clamp(targetDeg, 69.0, 78.0);
        hoodTarget = targetDeg;

        double motorRot = degreesToMotorRot(targetDeg);
        HoodMotor.setControl(positionRequest.withPosition(motorRot));
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

    @Override
    public void periodic() {
        double currentPos = getHoodPosition();

        SmartDashboard.putNumber("Hood Target", hoodTarget);
        SmartDashboard.putNumber("Hood Position", currentPos);
    }
}