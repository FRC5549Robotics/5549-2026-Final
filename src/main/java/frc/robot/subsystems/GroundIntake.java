package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Encoder;

public class GroundIntake extends SubsystemBase {

    private final SparkMax pivotMotor;
    //private final PIDController pivotPID;

    //private final SparkMax intakeMotor;
    private final TalonFX intakeMotor;
    //TalonFXConfiguration HoodMotorConfig;

    // Raw duty cycle input + decoded encoder (helps debug)
    private final DutyCycleEncoder pivotAbsEncoder;
    // tune these
    private double pivotEncoderOffsetRot = 0.0;           // 0..1 rotations offset
    private static final double ENC_TO_PIVOT_RATIO = 1.0; // gearing ratio if needed

    // continuous unwrap state
    //private double lastAdjAbsRot = 0.0;
    //private double continuousRot = 0.0;
    //private boolean absInit = false;

    private double pivotTargetRotations;

    // your existing setpoints (continuous rotations style)
    private final double PIVOT_UP_POSITION   = 145;
    private final double PIVOT_DOWN_POSITION = 268;

    private boolean pivotEnabled = false;

    private double upOrDown = 0;


    public GroundIntake() {
        pivotMotor = new SparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotAbsEncoder = new DutyCycleEncoder(
            Constants.PIVOT_ABS_ENC_DIO,
            360.0,   // full range = 360 degrees per rotation
            0.0      // zero position (can change later)

        );

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
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
        intakeMotor.getConfigurator().apply(intakeConfig);

        // Through Bore PWM signal into RoboRIO DIO
        
        

        //pivotPID = new PIDController(0.3, 0.0, 0.0);
        //pivotPID.setTolerance(0.05);

        pivotTargetRotations = 0.0;
    }

    // absolute encoder -> adjusted -> unwrapped continuous rotations -> scaled
    // absolute encoder -> adjusted -> unwrapped continuous rotations -> scaled
    public double getPivotPosition() {
        //System.out.println(pivotTargetRotations);
        // Returns absolute position in DEGREES (0–360)
        return pivotAbsEncoder.get();
    }

    public void setPivotUp() {
        //pivotPID.reset();
        upOrDown = 1;
        pivotTargetRotations = PIVOT_UP_POSITION;
        pivotEnabled = true;
        //intakeMotor.set(0.25);
        System.out.println(getPivotPosition());
        if (getPivotPosition() > 190) {
            System.out.println("up");
            pivotMotor.set(0.3);
        } else {
            pivotMotor.set(0.0);
        }

    }

    public void setPivotDown() {
        //pivotPID.reset();
        upOrDown = -1;
        pivotTargetRotations = PIVOT_DOWN_POSITION;
        intakeMotor.set(0.4);
        pivotEnabled = true;
        System.out.println(getPivotPosition());
        if (getPivotPosition() < 265) {
            System.out.println("down");
            pivotMotor.set(-0.3);
        } else {
            pivotMotor.set(0.0);
        }
    }

    public void shooting() {
        pivotEnabled = true;
        //pivotPID.reset();

        double pos = getPivotPosition();
        
        System.out.println(pos);

        if (pos < 190) {
            pivotMotor.set(-.2);
        } else if (pos > 205) {
            pivotMotor.set(.2);
        }
    }

    public void IntakeReverse() {
        intakeMotor.set(0.3);
    }

    //public void IntakeOff() {
        //intakeMotor.set(0.0);
    //}

    public void off() {
        pivotEnabled = false;
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
        //SmartDashboard.putNumber("GI Enc Degrees", raw * 360.0);
        SmartDashboard.putBoolean("GI Enc Raw!=0", raw != 0.0);
        double currentPos = getPivotPosition();
        SmartDashboard.putNumber("Pivot Pos", currentPos);
        SmartDashboard.putNumber("Pivot Target", pivotTargetRotations);
        SmartDashboard.putNumber("Up or down (up is 1, down is -1)", upOrDown);

        

    }
}
