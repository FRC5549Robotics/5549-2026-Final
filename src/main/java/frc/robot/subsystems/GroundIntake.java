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

    private final Extension m_Extension;

    // your existing setpoints (continuous rotations style)
    private final double PIVOT_UP_POSITION   = 145;
    private final double PIVOT_DOWN_POSITION = 268;

    private boolean holdingAtTop = false;

    private final VoltageOut voltageRequest = new VoltageOut(0).withEnableFOC(true);

    public GroundIntake(Extension extension) {
        this.m_Extension = extension;

        pivotMotor = new SparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotAbsEncoder = new DutyCycleEncoder(
            Constants.PIVOT_ABS_ENC_DIO,
            360.0,   // full range = 360 degrees per rotation
            0.0      // zero position (can change later)

        );

        SparkMaxConfig pivotConfig = new SparkMaxConfig();

        pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(15);

        pivotMotor.configure(
            pivotConfig,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters
        );

        intakeMotor = new TalonFX(Constants.GROUND_INTAKE_ID, "lil clanker");
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        intakeConfig.CurrentLimits.StatorCurrentLimit = 60;
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeConfig.CurrentLimits.SupplyCurrentLimit = 35;
        intakeConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.08; //0.08s to reach max voltage

        intakeMotor.getConfigurator().apply(intakeConfig);

    }

    private boolean extensionSafe() {
        return m_Extension.getPosition() > 10;
    }

    // absolute encoder -> adjusted -> unwrapped continuous rotations -> scaled
    // absolute encoder -> adjusted -> unwrapped continuous rotations -> scaled
    public double getPivotPosition() {
        //System.out.println(pivotTargetRotations);
        // Returns absolute position in DEGREES (0–360)
        return pivotAbsEncoder.get();
    }

    public void setPivotUp() {
        m_Extension.extend();
        if (getPivotPosition() > 220) {
            if (extensionSafe()) {
                pivotMotor.set(0.6);
            }
        } else {
            pivotMotor.set(0.0);
            intakeMotor.set(0.0);
            holdingAtTop = true;
            //System.out.println("All the way up");
        }
    }

    public void setPivotDown() {
        holdingAtTop = false;
        //intakeMotor.setControl(voltageRequest.withOutput(10)); //6
        //System.out.println("down called");
        if (getPivotPosition() < 275.5) {
            if (getPivotPosition() < 240) { //if intake is too far back, extend hopper before deploying intake
                m_Extension.extend();
                if (extensionSafe()) {
                    pivotMotor.set(-0.4);
                }
            } else {
                pivotMotor.set(-0.4); //if its already out enough, don't worry about the extension
            }
        } else {
            pivotMotor.set(0.0);
            intakeMotor.setControl(voltageRequest.withOutput(10));
        }
    }

    public void setPivotUpFully() {
        m_Extension.extend();
        if (getPivotPosition() > 170) {
            if (extensionSafe()) {
                pivotMotor.set(0.4);
            }
        } else {
            pivotMotor.set(0.0);
            intakeMotor.set(0.0);
            holdingAtTop = true;
        }
    }
    
    public void setPivotDownFast() {
        holdingAtTop = false;
        //intakeMotor.setControl(voltageRequest.withOutput(10)); //6
        if (getPivotPosition() < 275.5) {
            if (getPivotPosition() < 240) { //if intake is too far back, extend hopper before deploying intake
                m_Extension.extend();
                if (extensionSafe()) {
                    pivotMotor.set(-0.4);
                }
            } else {
                pivotMotor.set(-0.4); //if its already out enough, don't worry about the extension
            }
        } else {
            pivotMotor.set(0.0);
            intakeMotor.setControl(voltageRequest.withOutput(12));
        }
    }

    public void retractForExtension() {
        double pos = getPivotPosition();

        if (pos < 190 && pos > 170) { //if pivot is too far up but not all the way back
            pivotMotor.set(0.4); //fully retract the intake
        }
    }

    public void shooting() {
        m_Extension.extend();
        holdingAtTop = false;
        double pos = getPivotPosition();
        
        //System.out.println(pos);
        if (pos < 240) {
            if (extensionSafe()) {
                pivotMotor.set(-0.4);
            }
        } else if (pos > 250) {
            if (extensionSafe()) {
                pivotMotor.set(0.4);
            }
        }
    }

    public void IntakeReverse() {
        intakeMotor.set(-0.3);
    }

    public void IntakeOn(){
        //intakeMotor.set(.3);
    }

    public void off() {
        //System.out.println("pivotDisabled");
        intakeMotor.set(0.0);
        pivotMotor.set(0.0);
        holdingAtTop = false;
    }

    @Override
    public void periodic() {
        // ALWAYS push raw encoder debug so you can see if it's alive
        double raw = pivotAbsEncoder.get();

        SmartDashboard.putNumber("GI DIO Channel", Constants.PIVOT_ABS_ENC_DIO);
        SmartDashboard.putBoolean("GI Enc Connected", pivotAbsEncoder.isConnected());
        SmartDashboard.putNumber("GI Enc Degrees", raw);

        if (holdingAtTop) {
            pivotMotor.set(0.1);
        }
        
        SmartDashboard.putNumber("extension position", m_Extension.getPosition());
    }
}
