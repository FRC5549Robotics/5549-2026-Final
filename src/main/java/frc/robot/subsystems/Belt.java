package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;

public class Belt extends SubsystemBase{
    TalonFX belt_right;
    TalonFX belt_left;

    CommandXboxController Xboxcontroller;
    TalonFXConfiguration beltConfigs;
    TalonFXConfigurator beltConfigurator;

    private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0.0).withEnableFOC(true);
    
    //private final VoltageOut voltageRequest = new VoltageOut(0);

    //left follows right
    private final Follower belt_left_Follower = new Follower(Constants.BELT_RIGHT_MOTOR_ID, MotorAlignmentValue.Opposed);
    
    public Belt (){
        belt_right = new TalonFX(Constants.BELT_RIGHT_MOTOR_ID, "lil clanker");
        belt_left = new TalonFX(Constants.BELT_LEFT_MOTOR_ID, "lil clanker");

        beltConfigs = new TalonFXConfiguration();
        beltConfigurator = belt_right.getConfigurator();

        beltConfigs.CurrentLimits.StatorCurrentLimit = 60;
        beltConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        beltConfigs.CurrentLimits.SupplyCurrentLimit = 60;
        beltConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        beltConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        beltConfigs.Voltage.PeakForwardVoltage = 12.0;
        beltConfigs.Voltage.PeakReverseVoltage = -12.0;

        beltConfigs.Slot0.kP = 0.12;
        beltConfigs.Slot0.kI = 0.0;
        beltConfigs.Slot0.kD = 0.0;
        beltConfigs.Slot0.kV = 0.0;

        beltConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.08; //take 0.08 seconds to reach demanded output

        belt_right.getConfigurator().apply(beltConfigs);
        belt_left.getConfigurator().apply(beltConfigs);
        belt_left.setControl(belt_left_Follower);
    }

    
    public void intake(){
        belt_right.setControl(dutyCycleOut.withOutput(0.83)); 
    }
    public void jammed(){
        belt_right.setControl(dutyCycleOut.withOutput(-0.3)); //run belts backwards
    }
    
    public void off(){
        belt_right.setControl(dutyCycleOut.withOutput(0)); //turn off belts
    }
    
}
