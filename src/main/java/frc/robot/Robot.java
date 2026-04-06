// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import frc.robot.commands.ZeroExtension;
import frc.robot.commands.ZeroHood;
import frc.robot.shooter.ShooterLookup;
import frc.robot.util.GameState;
import frc.robot.util.PowerMonitor;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private PowerMonitor powerMonitor;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

       // CameraServer.startAutomaticCapture("camera", 0);

        for (int i = 1; i <= 6; i++) {
            // Use initDouble to set defaults only if they don't exist
            Preferences.initDouble("Shooter_Dist_" + i, 0.0);
            Preferences.initDouble("Shooter_Angle_" + i, 0.0);
            Preferences.initDouble("Shooter_RPM_" + i, 0.0);
            Preferences.initDouble("Shooter_Time_" + i, 0.0);
        }

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(200);

        DataLogManager.start();

        String[] channels = new String[25];
        channels[1] = "drive_front_left"; // port 1 = drive front left motor

        powerMonitor = new PowerMonitor(channels);
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        m_robotContainer.getGameState().update();

        //powerMonitor.log();
    }

    @Override
    public void disabledInit() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(200);
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        new ZeroHood(m_robotContainer.getHood()).schedule();

        new ZeroExtension(m_robotContainer.getExtension()).schedule();

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(0);

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }

        ShooterLookup.updateTableFromPreferences();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        new ZeroHood(m_robotContainer.getHood()).schedule();
        
        //new ZeroExtension(m_robotContainer.getExtension()).schedule();

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("throttle_set").setNumber(0);

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

        ShooterLookup.updateTableFromPreferences();
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}

}