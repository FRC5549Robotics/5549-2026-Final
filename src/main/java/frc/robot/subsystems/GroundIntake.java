package frc.robot.subsystems;

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
    private final PIDController pivotPID;

    private final SparkFlex intakeMotor;

    // Raw duty cycle input + decoded encoder (helps debug)
    private final DutyCycleEncoder pivotAbsEncoder;
    // tune these
    private double pivotEncoderOffsetRot = 0.0;           // 0..1 rotations offset
    private static final double ENC_TO_PIVOT_RATIO = 1.0; // gearing ratio if needed

    // continuous unwrap state
    private double lastAdjAbsRot = 0.0;
    private double continuousRot = 0.0;
    private boolean absInit = false;

    private double pivotTargetRotations;

    // your existing setpoints (continuous rotations style)
    private final double PIVOT_UP_POSITION   = 300;
    private final double PIVOT_DOWN_POSITION = 60;

    private boolean pivotEnabled = false;

    public GroundIntake() {
        pivotMotor = new SparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless);
        pivotAbsEncoder = new DutyCycleEncoder(
            Constants.PIVOT_ABS_ENC_DIO,
            360.0,   // full range = 360 degrees per rotation
            0.0      // zero position (can change later)
    
        
        );
                  // degrees

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kBrake);
        pivotMotor.configure(
            pivotConfig,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters
        );

        intakeMotor = new SparkFlex(Constants.GROUND_INTAKE_ID, MotorType.kBrushless);
        SparkFlexConfig intakeConfig = new SparkFlexConfig();
        intakeConfig.idleMode(IdleMode.kBrake);
        intakeMotor.configure(
            intakeConfig,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters
        );

        // Through Bore PWM signal into RoboRIO DIO
        
        

        pivotPID = new PIDController(0.3, 0.0, 0.0);
        pivotPID.setTolerance(0.05);

        pivotTargetRotations = 0.0;
    }

    // absolute encoder -> adjusted -> unwrapped continuous rotations -> scaled
    // absolute encoder -> adjusted -> unwrapped continuous rotations -> scaled
    public double getPivotPosition() {
        System.out.println("encoding " + pivotAbsEncoder.isConnected());
        // Returns absolute position in DEGREES (0–360)
        return pivotAbsEncoder.get();
    }




    public void setPivotUp() {
        pivotPID.reset();
        pivotTargetRotations = PIVOT_UP_POSITION;
        pivotEnabled = true;
        intakeMotor.set(0.25);
    }

    public void setPivotDown() {
        pivotPID.reset();
        pivotTargetRotations = PIVOT_DOWN_POSITION;
        intakeMotor.set(0.5);
        pivotEnabled = true;
    }

    public void shooting() {
        pivotEnabled = true;
        pivotPID.reset();

        double pos = getPivotPosition();

        if (pos < 100) {
            pivotTargetRotations = 200;
        } else if (pos > 100) {
            pivotTargetRotations = PIVOT_DOWN_POSITION;
        }
    }

    public void IntakeReverse() {
        intakeMotor.set(0.3);
    }

    public void IntakeOff() {
        intakeMotor.set(0.0);
    }

    public void off() {
        pivotEnabled = false;
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

        if (!pivotEnabled) {
            SmartDashboard.putNumber("Pivot Output", 0.0);
            return;
        }

        double output = pivotPID.calculate(currentPos, pivotTargetRotations);
        output = MathUtil.clamp(output, -0.15, 0.15);

        pivotMotor.set(output);
        SmartDashboard.putNumber("Pivot Output", output);
    }
}
