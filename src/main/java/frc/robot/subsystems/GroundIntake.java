package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

public class GroundIntake extends SubsystemBase{
    TalonFX pivotMotor;
    CANrange canRange;
    PIDController pivotPID;
    CommandXboxController Xboxcontroller;
    TalonFXConfiguration PivotConfigs;
    TalonFXConfigurator PivotConfigurator;
    ProfiledPIDController ElevatorController;

    SparkMax IntakeMotor;
    SparkMaxConfig IntakeConfig;

    private double pivotTargetRotations;
    private double PIVOT_UP_POSITION   = -3;
    private double PIVOT_DOWN_POSITION = -9.83;
    private boolean pivotEnabled = false;
   
    public GroundIntake (){
        pivotMotor = new TalonFX(Constants.PIVOT_MOTOR_ID);
        pivotTargetRotations = pivotMotor.getPosition().getValueAsDouble();

        pivotPID = new PIDController(.3, 0.0, 0.0);
        pivotPID.setTolerance(0.05);

        PivotConfigs = new TalonFXConfiguration();
        PivotConfigurator = pivotMotor.getConfigurator();
        // canRange = new CANrange(Constants.CANRANGE_ID);


        IntakeMotor = new SparkMax(Constants.GROUND_INTAKE_ID, MotorType.kBrushless);
        IntakeConfig = new SparkMaxConfig();
        IntakeConfig.idleMode(IdleMode.kBrake);
        IntakeMotor.configure(IntakeConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters );
        


        PivotConfigs.CurrentLimits.StatorCurrentLimit = 120;
        PivotConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        PivotConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        pivotMotor.getConfigurator().apply(PivotConfigs);
        
        
        
        
    }

   

    

    public void setPivotUp() {
        pivotPID.reset();
        pivotTargetRotations = PIVOT_UP_POSITION;
        pivotEnabled = true;
        IntakeMotor.set(.1);


    }

    public void setPivotDown() {
        pivotPID.reset();
        pivotTargetRotations = PIVOT_DOWN_POSITION;
        IntakeMotor.set(.1);
        pivotEnabled = true;
    }

    

    public void IntakeReverse(){
        IntakeMotor.set(.3);
    }
    public void IntakeOff(){
        IntakeMotor.set(0);
    }
    
    public void off (){
        
        pivotEnabled = false;
        IntakeMotor.set(0);
        pivotMotor.set(0);

    }

    // MOVES PIVOT DOWN WITHOUT RUNNING INTAKE
// public void pivotDownAuto() {
//     if(pivotMotor.getPosition().getValueAsDouble() < Constants.PIVOT_DOWN_POSITION){
//         pivotMotor.set(Constants.GROUND_PIVOT_SPEED);
//     } else {
//         pivotMotor.set(0);
//     }
// }

// MOVES PIVOT UP WITHOUT RUNNING INTAKE
// public void pivotUpAuto() {
//     if(pivotMotor.getPosition().getValueAsDouble() > Constants.PIVOT_UP_POSITION){
//         pivotMotor.set(-0.2);
//     } else {
//         pivotMotor.set(0);
//     }
// }

public boolean hasNote() {
    // Returns TRUE if the sensor sees an object, FALSE otherwise
    return canRange.getIsDetected().getValue();
}

// Add this to GroundIntake.java
public double getPivotPosition() {
    return pivotMotor.getPosition().getValueAsDouble();
}

    public void periodic(){

        double currentPos = pivotMotor.getPosition().getValueAsDouble();

        double output = pivotPID.calculate(currentPos, pivotTargetRotations);
        if (!pivotEnabled) {
            
            return;
        }

    // Clamp so we don’t send insane power
    output = MathUtil.clamp(output, -0.15, 0.15);

    pivotMotor.set(output);

    SmartDashboard.putNumber("Pivot Pos", currentPos);
    SmartDashboard.putNumber("Pivot Target", pivotTargetRotations);
    SmartDashboard.putNumber("Pivot Output", output);
    }


}
