package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix6.controls.VoltageOut;

public class Belt extends SubsystemBase{
    TalonFX belt; //Create belt motor
    TalonFXConfiguration beltConfigs;
    TalonFXConfigurator beltConfigurator;
    
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public Belt (){
    belt = new TalonFX(Constants.Belt_MOTOR_ID, "lil clanker"); //Define belt motor
    
    beltConfigs = new TalonFXConfiguration();
    beltConfigurator = belt.getConfigurator();
    beltConfigs.CurrentLimits.StatorCurrentLimit = 120; //Add current limit
    beltConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    beltConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast; //default to coast mode
    belt.getConfigurator().apply(beltConfigs);
    }
    
    public void intake() {
        belt.setControl(voltageRequest.withOutput(10)); //run belts
    }

    public void jammed() {
        belt.setControl(voltageRequest.withOutput(3)); //run belts backwards
    }
    
    public void off() {
        belt.setControl(voltageRequest.withOutput(0)); //turn off belts
    }
}
