package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;

public class Belt extends SubsystemBase{
    TalonFX belt_right;
    TalonFX belt_left;

    CommandXboxController Xboxcontroller;
    TalonFXConfiguration beltConfigs;
    TalonFXConfigurator beltConfigurator;

    private final TorqueCurrentFOC request = new TorqueCurrentFOC(0);
    
    //private final VoltageOut voltageRequest = new VoltageOut(0);

    //left follows right
    private final Follower belt_left_Follower = new Follower(Constants.BELT_RIGHT_MOTOR_ID, MotorAlignmentValue.Opposed);
    
    public Belt (){
        belt_right = new TalonFX(Constants.BELT_RIGHT_MOTOR_ID, "lil clanker");
        belt_left = new TalonFX(Constants.BELT_LEFT_MOTOR_ID, "lil clanker");

        beltConfigs = new TalonFXConfiguration();
        beltConfigurator = belt_right.getConfigurator();

        beltConfigs.CurrentLimits.StatorCurrentLimit = 120;
        beltConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        beltConfigs.CurrentLimits.StatorCurrentLimit = 40;
        beltConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        beltConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        belt_right.getConfigurator().apply(beltConfigs);
        belt_left.getConfigurator().apply(beltConfigs);
        belt_left.setControl(belt_left_Follower);
    }

    
    public void intake(){
        belt_right.setControl(request.withOutput(30)); //run belts
    }
    public void jammed(){
        belt_right.setControl(request.withOutput(-15)); //run belts backwards
    }
    
    public void off(){
        belt_right.setControl(request.withOutput(0)); //turn off belts
    }
    
}
