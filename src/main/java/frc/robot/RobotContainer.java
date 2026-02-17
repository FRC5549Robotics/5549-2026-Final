// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import javax.sound.sampled.Line;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import frc.robot.commands.AimAndSpinUpCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.Vision.LimelightHelpers;
import frc.robot.subsystems.LEDState;
import frc.robot.util.GameState;
import frc.robot.shooter.ShooterLookup;


public class RobotContainer {

    private final LinearFilter txFilter = LinearFilter.movingAverage(4);

    private final CommandXboxController m_driver = new CommandXboxController(0);
    private final CommandXboxController m_operator = new CommandXboxController(Constants.OPERATOR_CONTROLLER); //create op controller

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * Constants.DRIVER_DEADBAND).withRotationalDeadband(MaxAngularRate * Constants.DRIVER_DEADBAND) // Add a deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    JoystickButton resetOdometry = new JoystickButton(m_driver.getHID(), 8);
    
    POVButton lowerHood = new POVButton(m_operator.getHID(), 180);
    POVButton raiseHood = new POVButton(m_operator.getHID(), 0);

    private final Limelight m_limelight = new Limelight();
    private final GroundIntake m_pivot = new GroundIntake();
    private final Belt m_belt = new Belt();
    private final Shooter m_shooter = new Shooter();
    private final Hood m_hood = new Hood();
    private final LED m_LED = new LED();
    private final GameState gameState = new GameState();

    public void updateGameState() {
        gameState.update();
    }

    public Hood getHood() {
        return m_hood;
    }

    // AUTOCHOOSER SET UP
    private final SendableChooser<Command> autoChooser;

    public Command beltCommand(){
        return new WaitCommand(1.5).andThen(() -> m_belt.intake());
    }

    public Command intakeCommand(){
        return new WaitCommand(2).andThen(() -> new RunCommand(m_pivot::shooting));
    }

    public double getFilteredTX() {
        if (!LimelightHelpers.getTV("limelight")) {
            txFilter.reset();
            return 0.0;
        }

        double rawTx = LimelightHelpers.getTX("limelight");

        return txFilter.calculate(rawTx);
    }

    public RobotContainer() {

        ShooterLookup.initializeDashboard();
        ShooterLookup.loadFromFile();

        drivetrain.configurePathPlanner();  
        autoChooser = AutoBuilder.buildAutoChooser();
    //     autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        NamedCommands.registerCommand("RunBelt", new InstantCommand(m_belt::intake));
        NamedCommands.registerCommand("OffBelt", new InstantCommand(m_belt::off));
        NamedCommands.registerCommand("PivotDown", new InstantCommand(m_pivot:: setPivotDown));
        NamedCommands.registerCommand("PivotUp", new InstantCommand(m_pivot::setPivotUp));
        NamedCommands.registerCommand("ShootOff", new InstantCommand(m_shooter::off));
        configureBindings();
    }
    
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-m_driver.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //LIMELIGHT DRIVE LEFT BUMPER
        SwerveRequest.FieldCentric aimRequest = new SwerveRequest.FieldCentric();

        //While left bumper is held down, aim towards april tag, set hood, & spin up shooter
        m_driver.leftBumper()
            .whileTrue(
                new ParallelCommandGroup(
                    drivetrain.applyRequest(() -> {
                        double rotationOutput = 0;
                        double deadband = Constants.DRIVER_DEADBAND;

                        double translationX = MathUtil.applyDeadband(m_driver.getLeftY(), deadband);
                        double translationY = MathUtil.applyDeadband(m_driver.getLeftX(), deadband);

                        boolean hasTarget = LimelightHelpers.getTV("limelight");
                        //select correct pipeline for proper offsets
                        if (hasTarget) {

                            int tagID = (int) LimelightHelpers.getFiducialID("limelight");

                            int desiredPipeline = 0;

                            switch (tagID) {
                            // middle
                            case 5:
                            case 10:
                            case 2:
                            case 18:
                            case 26:
                            case 21:
                                desiredPipeline = 0;
                                break;
                            // right
                            case 8:
                            case 24:
                                desiredPipeline = 1;
                                break;
                            //left
                            case 9:
                            case 11:
                            case 25:
                            case 27:
                                desiredPipeline = 2;
                                break;
                            }

                            int currentPipeline = (int) LimelightHelpers.getCurrentPipelineIndex("limelight");

                            if (currentPipeline != desiredPipeline) {
                            LimelightHelpers.setPipelineIndex("limelight", desiredPipeline);
                            }

                            double tx = getFilteredTX();

                            double kP = Constants.AIM_kP;
                            rotationOutput = tx * -kP * MaxAngularRate;
                        } else {
                            rotationOutput = -m_driver.getRightX() * MaxAngularRate;
                        }

                        return aimRequest
                            .withVelocityX(-translationX * MaxSpeed)
                            .withVelocityY(-translationY * MaxSpeed)
                            .withRotationalRate(rotationOutput);
                    }),
                    new AimAndSpinUpCommand(m_limelight, m_shooter, m_hood)
                )
            )
            //When left bumper is released, turn off shooter, LED's show no longer ready to shoot
            .onFalse(
                new InstantCommand(m_shooter::off, m_shooter)
        );



        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driver.back().and(m_driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driver.back().and(m_driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driver.start().and(m_driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driver.start().and(m_driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        resetOdometry.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        //Intake up and down.
        m_operator.axisGreaterThan(1, Constants.TRIGGER_DEADBAND).whileTrue(new RunCommand(m_pivot::setPivotDown)).onFalse(new InstantCommand(m_pivot::off)); 
        m_operator.axisLessThan(1, -Constants.TRIGGER_DEADBAND).whileTrue(new RunCommand(m_pivot::setPivotUp)).onFalse(new InstantCommand(m_pivot::off)); 
        //Belt unjam
        m_operator.rightBumper().onTrue(new InstantCommand(m_belt:: jammed)).onFalse(new InstantCommand(m_belt:: off));
        //Operator shoots balls
        m_operator.axisGreaterThan(3, Constants.TRIGGER_DEADBAND)
            .whileTrue(
                new ParallelCommandGroup(

                    //Only run belts when shooter is up to speed
                    new RunCommand(() -> {
                        if (m_shooter.atSpeed()) {
                            m_belt.intake();
                        } else {
                            m_belt.off();
                        }
                    }, m_belt),

                    //X formation wheels
                    drivetrain.applyRequest(() -> brake),

                    // Wait 1.5 sec, then run shooting() continuously
                    new SequentialCommandGroup(
                        new WaitCommand(1.5),
                        new RunCommand(m_pivot::shooting, m_pivot)
                    )
                )
            )
            .onFalse(
                new ParallelCommandGroup(
                    new InstantCommand(m_belt::off, m_belt),
                    new InstantCommand(m_pivot::off, m_pivot)
                )
            );

        //setpoint for shooting close to hub
        m_operator.button(4).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> m_hood.setAngle(78.0), m_hood),
                new InstantCommand(() -> m_shooter.shoot(1860), m_shooter)
            )
        );
        //setpoint for passing
        m_operator.button(1).onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> m_hood.setAngle(69.0), m_hood),
                new InstantCommand(() -> m_shooter.shoot(1200), m_shooter)
            )
        );
    }

    public LEDState computeLEDState(
            boolean hubInactive,
            boolean lbHeld,
            double tx,
            boolean atSpeed) {

        if (hubInactive) {
            return LEDState.RED;
        }

        if (!lbHeld) {
            return LEDState.YELLOW;
        }

        if (Math.abs(tx) > 5 || !atSpeed) {
            return LEDState.YELLOW;
        }

        return LEDState.GREEN;
    }

    public void periodic() {

        gameState.update();

        boolean hubInactive = gameState.isHubInactiveNow();
        boolean lbHeld = m_driver.getHID().getLeftBumper();
        double tx = getFilteredTX();
        boolean atSpeed = m_shooter.atSpeed();

        LEDState state = computeLEDState(hubInactive, lbHeld, tx, atSpeed);

        m_LED.setState(state);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("polina");
    }
}
