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
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

public class InBelt extends SubsystemBase{
    TalonFX belt;
    CommandXboxController Xboxcontroller;
    TalonFXConfiguration beltConfigs;
    TalonFXConfigurator beltConfigurator;
    
    
    
    
    
    public InBelt (){
    belt = new TalonFX(Constants.Belt_MOTOR_ID, "lil clanker");
    
    beltConfigs = new TalonFXConfiguration();
    beltConfigurator = belt.getConfigurator();
        // canRange = new CANrange(Constants.CANRANGE_ID);


        


    beltConfigs.CurrentLimits.StatorCurrentLimit = 120;
    beltConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    beltConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    belt.getConfigurator().apply(beltConfigs);
    }

    
    public void intake(){
        belt.set(.9);
        System.out.println("belting");
    }
    public void jammed(){
        belt.set(-.2);
    }
    
    public void off(){
        belt.set(0);
    }
    
}
