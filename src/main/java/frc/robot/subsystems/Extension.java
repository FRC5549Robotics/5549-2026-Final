package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class Extension extends SubsystemBase{

    private boolean PIDEnabled = false; //default to PID being off

    private final PIDController extensionPID = new PIDController(0.1, 0.0, 0.004); // kP, kI, kD
    private static final double kS = 0.12;

    private double extensionSetpoint = 0;

    {
        extensionPID.setTolerance(1.2); //1 motor rotation of tolerance = 1/9 bottom pulley tolerance
    }

    //private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withEnableFOC(true);
    private final DutyCycleOut homingRequest = new DutyCycleOut(-0.085);

    TalonFX ExtensionMotor;
    TalonFXConfiguration ExtensionMotorConfig;

    public Extension(){
        ExtensionMotor = new TalonFX(Constants.EXTENSION_MOTOR_ID, "lil clanker");
        ExtensionMotorConfig = new TalonFXConfiguration();

        ExtensionMotorConfig.CurrentLimits.StatorCurrentLimit = 50;
        ExtensionMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        ExtensionMotorConfig.CurrentLimits.SupplyCurrentLimit = 35; 
        ExtensionMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        ExtensionMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        ExtensionMotorConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        ExtensionMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.05;

        ExtensionMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 4; // cap on velocity
        ExtensionMotorConfig.MotionMagic.MotionMagicAcceleration = 8; // cap on acceleration

        ExtensionMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        ExtensionMotor.getConfigurator().apply(ExtensionMotorConfig);

    }

    public void extend() {
        extensionSetpoint = 21.5;
    }

    public void retract() {
        extensionSetpoint = 0;
        System.out.println("hopper retracted");
    }

    public boolean atTarget() {
        return extensionPID.atSetpoint();
    }

    public void extensionDownSlow() {
        extensionSetpoint = 0;
        ExtensionMotor.setControl(homingRequest);
    }

    //public void stop() {
        //ExtensionMotor.setControl(new NeutralOut());
        //System.out.println("extension motor stop() ran");
        //PIDEnabled = true;
    //}

    public void zeroEncoder() {
        ExtensionMotor.setControl(new NeutralOut());
        ExtensionMotor.setPosition(0);
        PIDEnabled = true;
        System.out.println("PID enabled = true");
    }

    public boolean atBottom() {
        //boolean velocitySlow = Math.abs(ExtensionMotor.getVelocity().getValueAsDouble()) < 0.05;
        //SmartDashboard.putBoolean("velocitySlow", velocitySlow);

        //boolean currentSpiked = ExtensionMotor.getStatorCurrent().getValueAsDouble() > 25.0;
        //SmartDashboard.putNumber("current", ExtensionMotor.getStatorCurrent().getValueAsDouble());

        //return velocitySlow && currentSpiked;

        return true; //just assume its at the bottom and we put it all the way down
    }

    public boolean atBottomTeleop() {
        boolean velocitySlow = Math.abs(ExtensionMotor.getVelocity().getValueAsDouble()) < 0.05;
        //SmartDashboard.putBoolean("velocitySlow", velocitySlow);

        boolean currentSpiked = ExtensionMotor.getStatorCurrent().getValueAsDouble() > 25.0;
        //SmartDashboard.putNumber("current", ExtensionMotor.getStatorCurrent().getValueAsDouble());

        return velocitySlow && currentSpiked;
    }

    public double getPosition() {
        return ExtensionMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {
        double currentPos = ExtensionMotor.getPosition().getValueAsDouble();

        //SmartDashboard.putNumber("extension Target", extensionSetpoint);
        //SmartDashboard.putNumber("extension Position", currentPos);
        //SmartDashboard.putBoolean("extension PID enabled", PIDEnabled);

        if (!PIDEnabled) return;

        double output = extensionPID.calculate(currentPos, extensionSetpoint);

        //SmartDashboard.putNumber("Hood Output", output);

        if (Math.abs(output) > 0.001) {
            output += Math.signum(output) * kS;
        }

        output = MathUtil.clamp(output, -0.4, 0.4);

        if (atTarget()) {
            //if (extensionSetpoint == 20 && currentPos < 19.5) {
                //ExtensionMotor.setControl(new DutyCycleOut(0.1).withEnableFOC(true));
            //} else {
                ExtensionMotor.setControl(new NeutralOut());
            //}
        } else {
            ExtensionMotor.setControl(new DutyCycleOut(output).withEnableFOC(true)); //move the hood
        }
    }
}