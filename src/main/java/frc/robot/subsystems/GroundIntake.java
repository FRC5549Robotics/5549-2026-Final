package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
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
import com.ctre.phoenix6.controls.VoltageOut;

public class GroundIntake extends SubsystemBase {

    private final SparkMax pivotMotor; // Create pivotMotor as a sparkMax device
    private final TalonFX intakeMotor; //Create intakeMotor as a talonFX device
    private final DutyCycleEncoder pivotAbsEncoder; //create pivotAbsEncoder as a DutyCycleEncoder

    private double pivotTargetPosition; //Create variable for where we want pivot to be

    //Maximum and minimum positions
    private final double PIVOT_UP_POSITION   = 145;
    private final double PIVOT_DOWN_POSITION = 268;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    public GroundIntake() {

        pivotMotor = new SparkMax(Constants.PIVOT_MOTOR_ID, MotorType.kBrushless); //Further define pivotmotor
        pivotAbsEncoder = new DutyCycleEncoder(
            Constants.PIVOT_ABS_ENC_DIO, //DIO port
            360.0,   // One rotation is 360
            0.0      // How much to offset the encoder

        );

        SparkMaxConfig pivotConfig = new SparkMaxConfig();
        pivotConfig.idleMode(IdleMode.kCoast); //set default mode to coast mode
        pivotMotor.configure(
            pivotConfig,
            com.revrobotics.ResetMode.kResetSafeParameters,
            com.revrobotics.PersistMode.kPersistParameters
        );

        intakeMotor = new TalonFX(Constants.GROUND_INTAKE_ID, "lil clanker"); //Further define intakeMotor
        TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
        intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast; //set deault mode to coast mode
        intakeConfig.CurrentLimits.StatorCurrentLimit = 60; //Intake current limit
        intakeConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        intakeMotor.getConfigurator().apply(intakeConfig);
    }

    public double getPivotPosition() {
        return pivotAbsEncoder.get(); //returns absolute encoder position
    }

    public void setPivotUp() {
        pivotTargetPosition = PIVOT_UP_POSITION; //pivot wants to go to upper setpoint
        if (getPivotPosition() > 190) { // if its not at max height then...
            pivotMotor.set(0.3); // move it up
        } else {
            pivotMotor.set(0.0); // if it is at max height, then stop
        }
    }

    public void setPivotDown() {
        pivotTargetPosition = PIVOT_DOWN_POSITION; //pivot wants to go to lower setpoint
        intakeMotor.setControl(voltageRequest.withOutput(4)); // turn intake motor on
        if (getPivotPosition() < 265) { // if its not at min height then...
            pivotMotor.set(-0.3); // send it down
        } else {
            pivotMotor.set(0.0);  //if it is at min height, then stop
        }
    }

    public void shooting() { //we run the intake up and down when we're shooting to push the balls in
        double pos = getPivotPosition(); //get the pivot position

        if (pos < 190) { // if its less than 190...
            pivotMotor.set(-.2); //move it down
        } else if (pos > 205) { // if its more than 205...
            pivotMotor.set(.2); //move it up
        }
    }

    public void off() {
        intakeMotor.set(0.0); //stop running intake
        pivotMotor.set(0.0); //stop running pivot
    }

    @Override
    public void periodic() {

        double raw = pivotAbsEncoder.get(); //get encoder value

        SmartDashboard.putBoolean("GI Enc Connected", pivotAbsEncoder.isConnected()); //Is the encoder connected?
        SmartDashboard.putNumber("GI Enc Degrees", raw); //What is the encoder at?
        SmartDashboard.putNumber("Pivot Target", pivotTargetPosition); //Where do we want the pivot to be?

    }
}