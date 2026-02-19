package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase {

    private final SparkMax pivotMotor;
    private final TalonFX intakeMotor;
    private final DutyCycleEncoder pivotAbsEncoder;

    private double pivotTargetRotations;

    // your existing setpoints (continuous rotations style)
    private final double PIVOT_UP_POSITION   = 145;
    private final double PIVOT_DOWN_POSITION = 268;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

    public GroundIntake() {
        pivotMotor = new SparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotAbsEncoder = new DutyCycleEncoder(
            Constants.PIVOT_ABS_ENC_DIO,
            360.0,   // full range = 360 degrees per rotation
            0.0      // zero position (can change later)

        );

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kCoast);
        pivotMotor.configure(
            pivotConfig,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters
        );

        intakeMotor = new TalonFX(Constants.GROUND_INTAKE_ID, "lil clanker");
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 35;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.08; //0.08s to reach max voltage

        intakeMotor.getConfigurator().apply(intakeConfig);

    }

    // absolute encoder -> adjusted -> unwrapped continuous rotations -> scaled
    // absolute encoder -> adjusted -> unwrapped continuous rotations -> scaled
    public double getPivotPosition() {
        //System.out.println(pivotTargetRotations);
        // Returns absolute position in DEGREES (0–360)
        return pivotAbsEncoder.get();
    }

    public void setPivotUp() {
        if (getPivotPosition() > 190) {
            pivotMotor.set(0.3);
        } else {
            pivotMotor.set(0.0);
        }
    }

    public void setPivotDown() {
        intakeMotor.setControl(voltageRequest.withOutput(4.5));
        if (getPivotPosition() < 265) {
            pivotMotor.set(-0.3);
        } else {
            pivotMotor.set(0.0);
        }
    }

    public void shooting() {
        double pos = getPivotPosition();
        
        System.out.println(pos);
        if (pos < 200) {
            pivotMotor.set(-.2); //go down
        } else if (pos > 215) {
            pivotMotor.set(.2);
        }
    }

    public void IntakeReverse() {
        intakeMotor.set(0.3);
    }

    public void off() {
        //System.out.println(pivotEnabled);
        intakeMotor.set(0.0);
        pivotMotor.set(0.0);
    }

    @Override
    public void periodic() {
        // ALWAYS push raw encoder debug so you can see if it's alive
        double raw = pivotAbsEncoder.get();

        SmartDashboard.putNumber("GI DIO Channel", Constants.PIVOT_ABS_ENC_DIO);
        SmartDashboard.putBoolean("GI Enc Connected", pivotAbsEncoder.isConnected());
        SmartDashboard.putNumber("GI Enc Degrees", raw);

    }
}
