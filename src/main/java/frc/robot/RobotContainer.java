// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AimAndSpinUpCommand;
import frc.robot.commands.AlignLimelight;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.TeleopShootCommand;
import frc.robot.commands.PassCommand;
import frc.robot.commands.ZeroExtensionTeleop;
import frc.robot.commands.ZeroHood;
import frc.robot.commands.ZeroExtensionInstant;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.Shooter;
import frc.robot.util.GameState;
// import frc.robot.subsystems.LED;
import frc.robot.subsystems.Extension;
import frc.robot.subsystems.Limelight;
import frc.robot.Vision.LimelightHelpers;
// import frc.robot.subsystems.LEDState;

public class RobotContainer {

     private final LinearFilter txFilter = LinearFilter.movingAverage(4);

    private final CommandXboxController m_operator = new CommandXboxController(Constants.OPERATOR_CONTROLLER);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    //private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driver = new CommandXboxController(0);
    JoystickButton AutoAlign = new JoystickButton(m_driver.getHID(), 3);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    private final RobotStateEstimator stateEstimator = new RobotStateEstimator(drivetrain);

    JoystickButton resetOdometry = new JoystickButton(m_driver.getHID(), 8);
    JoystickButton groundIntakeShakeButton = new JoystickButton(m_operator.getHID(), 6);
    
    
    // JoystickButton AutoAlignLeft = new JoystickButton(joystick.getHID(), 3);
    // JoystickButton AutoAlignRight = new JoystickButton(joystick.getHID(), 2);
    
    //POVButton lowerHood = new POVButton(m_operator.getHID(), 180);
    //POVButton raiseHood = new POVButton(m_operator.getHID(), 0);

    private final Limelight m_limelight = new Limelight(drivetrain, m_driver);
    private final Extension m_extension = new Extension();
    private final GroundIntake m_pivot = new GroundIntake(m_extension);
    private final Belt m_belt = new Belt();
    private final Shooter m_shooter = new Shooter();
    private final Hood m_hood = new Hood();
    // private final LED m_LED = new LED();
    // private final Candle m_leds = new Candle();
    private final GameState gameState = new GameState();

    private int lastPipeline = -1;

    public double getFilteredTX() {
        if (!LimelightHelpers.getTV("limelight")) {
            txFilter.reset();
            return Double.NaN;
        }

        double rawTx = LimelightHelpers.getTX("limelight");

        return txFilter.calculate(rawTx);
    }

    public Hood getHood() {
        return m_hood;
    }

    public Extension getExtension() {
        return m_extension;
    }

    // AUTOCHOOSER SET UP
    private SendableChooser<Command> autoChooser;

    public Command beltCommand(){
        return new WaitCommand(0.5).andThen(() -> m_belt.intake());
    }

    public Command intakeCommand(){
        return new WaitCommand(1.5).andThen(() -> new RunCommand(m_pivot::shooting));
    }
    public RobotContainer() {
        drivetrain.configurePathPlanner();  
        
    //     autoChooser = AutoBuilder.buildAutoChooser();
        
        NamedCommands.registerCommand("RunBelt", new InstantCommand(m_belt::intake));
        NamedCommands.registerCommand("WaitAndBelt", beltCommand());
        NamedCommands.registerCommand("PivotUp", new RunCommand(
                () -> m_pivot.setPivotUp(), //what to run while active
                m_pivot
            ).withTimeout(2.15) //how long to run it
        );
        NamedCommands.registerCommand("OffBelt", new InstantCommand(m_belt::off));
        NamedCommands.registerCommand("PivotDownAndIntake", new RunCommand(
                () -> m_pivot.setPivotDownFast(), //what to run while active
                m_pivot
            ).withTimeout(5) //how long to run it
        );
        NamedCommands.registerCommand("PivotDownAndIntake7s", new RunCommand(
                () -> m_pivot.setPivotDownFast(), //what to run while active
                m_pivot
            ).withTimeout(7) //how long to run it
        );

        NamedCommands.registerCommand("PivotUp", new InstantCommand(m_pivot::setPivotUp));
        NamedCommands.registerCommand("shoot", new InstantCommand(() -> m_shooter.shoot(1800), m_shooter));
        NamedCommands.registerCommand("ShootOff", new InstantCommand(m_shooter::off));
        NamedCommands.registerCommand("HoodTo78", new InstantCommand(() -> m_hood.setAngle(78.0), m_hood));
        NamedCommands.registerCommand("AimAndSpinUp", new AimAndSpinUpCommand(m_limelight, m_shooter, m_hood));
        NamedCommands.registerCommand(
            "AutoShootCommand",
            new SequentialCommandGroup(
                //new ZeroHood(m_hood),
                new ParallelCommandGroup(new InstantCommand(m_pivot:: IntakeOn), new AutoShootCommand(drivetrain, m_limelight, m_shooter, m_hood, m_belt, m_pivot)
                .withTimeout(15))
                
            )
        );
        NamedCommands.registerCommand(
            "QuickAutoShootCommand",
            new SequentialCommandGroup(
                //new ZeroHood(m_hood),
                new ParallelCommandGroup(new InstantCommand(m_pivot:: IntakeOn), new AutoShootCommand(drivetrain, m_limelight, m_shooter, m_hood, m_belt, m_pivot)
                .withTimeout(7))
                
            )
        );
        NamedCommands.registerCommand(
            "4sQuickAutoShootCommand",
            new SequentialCommandGroup(
                //new ZeroHood(m_hood),
                new ParallelCommandGroup(new InstantCommand(m_pivot:: IntakeOn), new AutoShootCommand(drivetrain, m_limelight, m_shooter, m_hood, m_belt, m_pivot)
                .withTimeout(4))
                
            )
        );
        NamedCommands.registerCommand("RetractHopper", new InstantCommand(m_extension::retract));

        NamedCommands.registerCommand("ExtendHopper", new InstantCommand(m_extension::extend));

        NamedCommands.registerCommand("ZeroHood", new InstantCommand(() -> new ZeroHood(m_hood).schedule()));

        NamedCommands.registerCommand("ZeroExtension", new InstantCommand(() -> new ZeroExtensionInstant(m_extension).schedule()));

        //autoChooser = AutoBuilder.buildAutoChooser();
        //SmartDashboard.putData("Auto Chooser", autoChooser);

        SendableChooser<Boolean> flipChooser = new SendableChooser<>();
        flipChooser.setDefaultOption("Not Flipped", false);
        flipChooser.addOption("Flipped", true);
        SmartDashboard.putData("Flip Chooser", flipChooser);

        flipChooser.onChange((Boolean flip) -> {
            autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                autoStream -> autoStream.map(auto -> new PathPlannerAuto(auto.getName(), flip))
            );
            SmartDashboard.putData("Auto Chooser", autoChooser);
        });

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            autoStream -> autoStream.map(
                auto -> new PathPlannerAuto(auto.getName(), flipChooser.getSelected())
            )
        );
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // m_LED.setStateSupplier(() -> {

        //     boolean hubInactive = gameState.isHubInactiveNow();
        //     boolean rbHeld = m_driver.getHID().getRightBumper();
        //     double tx = getFilteredTX();
        //     boolean atSpeed = m_shooter.atSpeed();
        //     double seconds = gameState.getSecondsUntilHubToggle();

        //     m_LED.setCountdown(seconds);

        //     //SmartDashboard.putBoolean("Hub Inactive", hubInactive);
        //     //SmartDashboard.putBoolean("LB Held", rbHeld);
        //     //SmartDashboard.putNumber("Limelight TX", tx);
        //     //SmartDashboard.putBoolean("Shooter At Speed", atSpeed);

        //     return computeLEDState(hubInactive, rbHeld, tx, atSpeed);
        
        // });

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

        m_driver.rightBumper()
            .whileTrue(
                new ParallelCommandGroup(
                new TeleopShootCommand(drivetrain, m_limelight, m_shooter, m_hood, m_belt, m_pivot, () -> !(m_operator.getRawAxis(1) > Constants.TRIGGER_DEADBAND && groundIntakeShakeButton.getAsBoolean())))
            )
            .onFalse(
                new InstantCommand(m_shooter::off, m_shooter)
            );



        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //m_driver.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driver.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driver.getLeftY(), -m_driver.getLeftX()))
        ));

        
        // Run SysId routines

        // Note that each routine should be run exactly once in a single log.
        m_driver.back().and(m_driver.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driver.back().and(m_driver.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driver.start().and(m_driver.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driver.start().and(m_driver.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        m_driver.axisGreaterThan(3, .7).whileTrue(new RunCommand(m_pivot:: setPivotUp )).onFalse(new InstantCommand(m_pivot:: off));
         m_driver.axisGreaterThan(2, .7).whileTrue(new RunCommand(m_pivot:: setPivotUpFully )).onFalse(new InstantCommand(m_pivot:: off));
         m_driver.button(5).whileTrue(new RunCommand(m_pivot::setPivotDown)).onFalse(new InstantCommand(m_pivot:: off));

        //m_driver.pov(0).whileTrue(Commands.run(() -> m_shooter.runCharacterization(2.0), m_shooter));
        //m_driver.pov(180).whileTrue(Commands.run(() -> m_shooter.runCharacterization(8.0), m_shooter));


        // Reset the field-centric heading on left bumper press.
        //resetOdometry.onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
        resetOdometry.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.fromDegrees(180))));

        drivetrain.registerTelemetry(logger::telemeterize);
        // joystick.povRight().onTrue(new AlignLimelight(true, drivetrain).withTimeout(3));



        // groundIntakeButton.whileTrue(new InstantCommand(m_pivot :: pivotDown)).onFalse(new InstantCommand(m_pivot::off));
        
        //operator intake controls
        m_operator.axisGreaterThan(5, Constants.TRIGGER_DEADBAND).whileTrue(new RunCommand(m_pivot::setPivotDown, m_pivot)).onFalse(new InstantCommand(m_pivot::off, m_pivot)); 
        m_operator.axisLessThan(5, -Constants.TRIGGER_DEADBAND).whileTrue(new RunCommand(m_pivot::setPivotUp, m_pivot)).onFalse(new InstantCommand(m_pivot::off, m_pivot)); 

        m_operator.axisGreaterThan(1, Constants.TRIGGER_DEADBAND).whileTrue(new RunCommand(m_pivot::setPivotDown, m_pivot)).onFalse(new InstantCommand(m_pivot::off, m_pivot)); 
        m_operator.axisLessThan(1, -Constants.TRIGGER_DEADBAND).whileTrue(new RunCommand(m_pivot::setPivotUp, m_pivot)).onFalse(new InstantCommand(m_pivot::off, m_pivot)); 
        //groundIntakeShakeButton.whileTrue(new RunCommand(m_pivot::shooting, m_pivot)); //RB to shake
        //Belt unjam
        m_operator.leftBumper().onTrue(new InstantCommand(m_belt:: jammed)).onFalse(new InstantCommand(m_belt:: off));
        //Operator shoots balls
        /* m_operator.axisGreaterThan(3, Constants.TRIGGER_DEADBAND)
            .whileTrue(
                new ParallelCommandGroup(
                    new RunCommand(() -> m_shooter.shoot(2300), m_shooter), //2300
                    new SequentialCommandGroup(
                        new WaitCommand(0.2),
                        new RunCommand(m_belt::intake, m_belt)
                    )
                )
            )
            .onFalse(
                new ParallelCommandGroup(
                    new InstantCommand(m_shooter::off, m_shooter),
                    new InstantCommand(m_belt::off, m_belt)
                )
            ); */

        m_operator.axisGreaterThan(3, Constants.TRIGGER_DEADBAND)
            .whileTrue(
                new PassCommand(drivetrain, m_shooter, m_hood, m_belt)
            );

        /* m_operator.axisGreaterThan(3, Constants.TRIGGER_DEADBAND)
            .onTrue(
                new InstantCommand(() -> m_hood.setAngle(69), m_hood)
            ); */

        m_operator.axisGreaterThan(2, Constants.TRIGGER_DEADBAND)
            .whileTrue(new RunCommand(m_pivot::IntakeReverse, m_pivot))
            .onFalse(new InstantCommand(m_pivot::off, m_pivot));


        //setpoint for shooting close to hub
        m_operator.button(4).onTrue(new InstantCommand(() -> m_hood.setAngle(78.0), m_hood));

        m_operator.button(4).whileTrue(new RunCommand(() -> m_shooter.shoot(1760), m_shooter));
        
        m_operator.button(4).onFalse(new InstantCommand(() -> m_shooter.off(), m_shooter));

        //setpoint for testing
        m_operator.button(1).onTrue(new InstantCommand(() -> m_hood.setAngle(52), m_hood));
        m_operator.button(1).whileTrue(new RunCommand(() -> m_shooter.shoot(1000), m_shooter));
        m_operator.button(1).whileTrue(new RunCommand(() -> m_belt.intake(), m_belt));
        m_operator.button(1).onFalse(new InstantCommand(() -> m_shooter.off(), m_shooter));
        m_operator.button(1).onFalse(new InstantCommand(() -> m_belt.off(), m_belt));

        m_operator.pov(0).or(m_operator.pov(45)).or(m_operator.pov(315)).onTrue(new InstantCommand(m_extension::extend, m_extension));
        m_operator.pov(180).or(m_operator.pov(135)).or(m_operator.pov(225))
            .onTrue(new ParallelCommandGroup(
                new InstantCommand(m_pivot::retractForExtension, m_pivot),
                new InstantCommand(m_extension::retract, m_extension)
            ));

        m_operator.button(8).onTrue(new ZeroExtensionTeleop(m_extension));
        m_operator.button(7).onTrue(new ZeroHood(m_hood));
    }

    public GameState getGameState() {
        return gameState;
    }


    // public LEDState computeLEDState(
    //     boolean hubInactive,
    //     boolean rbHeld,
    //     double tx,
    //     boolean atSpeed) {

    //     if (hubInactive) {
    //         return LEDState.RED;
    //     }

    //     if (!rbHeld) {
    //         return LEDState.PURPLE;
    //     }

    //     //if (!Double.isFinite(tx) || Math.abs(tx) > 5) {
    //         //return LEDState.PURPLE;
    //     //}

    //      return LEDState.GREEN;
    // }


    public Command getAutonomousCommand() {

        System.out.println("getAutonomousCommand");
        return autoChooser.getSelected();
    }
}
