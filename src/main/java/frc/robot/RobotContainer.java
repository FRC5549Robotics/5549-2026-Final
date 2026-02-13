// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AlignLimelight;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.subsystems.GroundIntake;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.InBelt;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Limelight;
import frc.robot.Vision.LimelightHelpers;


public class RobotContainer {

  private final CommandXboxController m_controller2 = new CommandXboxController(Constants.OPERATOR_CONTROLLER);

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    JoystickButton AutoAlign = new JoystickButton(joystick.getHID(), 3);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    
    JoystickButton groundIntakeButton = new JoystickButton(m_controller2.getHID(), 6);
    
    // JoystickButton AutoAlignLeft = new JoystickButton(joystick.getHID(), 3);
    // JoystickButton AutoAlignRight = new JoystickButton(joystick.getHID(), 2);
    
    POVButton lowerHood = new POVButton(m_controller2.getHID(), 180);
    POVButton raiseHood = new POVButton(m_controller2.getHID(), 0);

    private final Limelight m_limelight = new Limelight(drivetrain, joystick);
    // private final Intake m_intake = new Intake();
    // private final Shooter m_Shooter = new Shooter();
    // private final Belt m_Belt = new Belt();
    private final GroundIntake m_pivot = new GroundIntake();
    private final InBelt m_belt = new InBelt();
    private final Shooter m_shooter = new Shooter();
    private final Hood m_hood = new Hood();
    private final LED m_Led = new LED();
    // private final Candle m_leds = new Candle();

    
    // AUTOCHOOSER SET UP
    private final SendableChooser<Command> autoChooser;
    public Command beltCommand(){
        return new WaitCommand(1.5).andThen(() -> m_belt.intake());
    }

    public Command intakeCommand(){
        return new WaitCommand(2).andThen(() -> new RunCommand(m_pivot::shooting));
    }
    public RobotContainer() {
        drivetrain.configurePathPlanner();  
        autoChooser = AutoBuilder.buildAutoChooser();
    //     autoChooser = AutoBuilder.buildAutoChooser();
        
        
        SmartDashboard.putData("Auto Chooser", autoChooser);
        NamedCommands.registerCommand("RunBelt", new InstantCommand(m_belt::intake));
        NamedCommands.registerCommand("OffBelt", new InstantCommand(m_belt::off));
        NamedCommands.registerCommand("PivotDown", new InstantCommand(m_pivot:: setPivotDown));
        NamedCommands.registerCommand("PivotUp", new InstantCommand(m_pivot::setPivotUp));
        NamedCommands.registerCommand("Shoot1", new InstantCommand(m_shooter::shoot1));
        NamedCommands.registerCommand("ShootOff", new InstantCommand(m_shooter::off));
        configureBindings();
    //     NamedCommands.registerpCommand("RunBelt", new InstantCommand(m_Belt::runBelt));
    //     NamedCommands.registerCommand("StopBelt", new InstantCommand(m_Belt::off));
    //     NamedCommands.registerCommand("ShootHigh", new InstantCommand(m_Shooter::shootHigh));
    //     NamedCommands.registerCommand("Intake", new InstantCommand(m_intake::intake));

    //       // 1. COMPLEX SHOT COMMAND
    //     // Logic: Belt runs for 3.7s total. At 0.7s mark, Shooter/Intake join in for 3.0s.
    //     NamedCommands.registerCommand("ComplexShot", 
    //         Commands.parallel(
    //             // PROCESS A: The Belt (Runs for the whole duration: 0.7 delay + 3.0 shoot)
    //             // We use startEnd to ensure it turns OFF automatically when the timeout finishes
    //             // Commands.startEnd(m_Belt::runBelt, m_Belt::off, m_Belt)
    //             //     .withTimeout(4.0),
    //             Commands.startEnd(m_Shooter::shootHigh, m_Shooter::off, m_Shooter)
    //                 .withTimeout(4),

    //             // PROCESS B: The Delays and other systems
    //             Commands.sequence(
    //             // 1. UNJAM: Reverse intake briefly to pull note away from flywheels
    //             Commands.startEnd(m_intake::reverse, m_intake::off, m_intake)
    //                 .withTimeout(0.4),

    //             // 2. WAIT: Continue waiting for shooter spin-up
    //             // We reduced this from 1.0 to 0.7 because the unjam took 0.3s
    //             // (0.3s + 0.7s = 1.0s Total Prep Time)
    //             new WaitCommand(0.4), 
                
    //             // 3. FIRE: Run Belt and Intake forward to feed the shooter
    //             Commands.parallel(
    //                 Commands.startEnd(m_Belt::runBelt, m_Belt::off, m_Belt),
    //                 Commands.startEnd(m_intake::intake, m_intake::off, m_intake)
    //             ).withTimeout(3.2))
    //         )
    //     );

    //     NamedCommands.registerCommand("QuickShot", 
    //     Commands.parallel(
    //         // PROCESS A: The Belt (Runs for the whole duration: 0.7 delay + 3.0 shoot)
    //         // We use startEnd to ensure it turns OFF automatically when the timeout finishes
    //         // Commands.startEnd(m_Belt::runBelt, m_Belt::off, m_Belt)
    //         //     .withTimeout(4.0),
    //         Commands.startEnd(m_Shooter::shootHigh, m_Shooter::off, m_Shooter)
    //             .withTimeout(4),

    //         // PROCESS B: The Delays and other systems
    //         Commands.sequence(
    //           Commands.startEnd(m_intake::reverse, m_intake::off, m_intake)
    //             .withTimeout(0.4),

    //       // 2. WAIT: Continue waiting for shooter spin-up
    //       // We reduced this from 1.0 to 0.7 because the unjam took 0.3s
    //       // (0.3s + 0.7s = 1.0s Total Prep Time)
    //           new WaitCommand(0.5), 
                
    //             // Now start the Shooter and Intake together
    //           // Commands.parallel(
    //           //     Commands.startEnd(m_Belt::runBelt, m_Belt::off, m_Belt),
    //           //     Commands.startEnd(m_intake::intake, m_intake::off, m_intake)
    //           //   ).withTimeout(1.9), // They run for 3 seconds then stop

    //           // Commands.startEnd(m_intake::reverse, m_intake::off, m_intake)
    //           //   .withTimeout(0.3),
    //           Commands.parallel(
    //           Commands.startEnd(m_Belt::runBelt, m_Belt::off, m_Belt),
    //           Commands.startEnd(m_intake::intake, m_intake::off, m_intake)
    //           ).withTimeout(3.1) // They run for 3 seconds then stop
    //         )
    //     )
    //     );

    //     // 2. GROUND INTAKE CYCLE
    //     // Logic: Pivot down, run intake, pivot up.
    //     // NamedCommands.registerCommand("GroundIntakeCycle", 
    //     //     Commands.sequence(
    //     //         // Step 1: Pivot Down (Assuming this is instant or needs a small wait to settle)
    //     //         Commands.runOnce(m_pivot::pivotDown, m_pivot),
    //     //         new WaitCommand(0.5), // Give it 0.5s to actually physically lower
                
    //     //         // Step 2: Run the intake rollers for 2.0 seconds (or however long you need)
    //     //         Commands.startEnd(m_pivot::IntakeOn, m_pivot::off, m_pivot)
    //     //             .withTimeout(2.0),

    //     //         // Step 3: Pivot back up
    //     //         Commands.runOnce(m_pivot::pivotUp, m_pivot)
    //     //     )
    //     // );

    //     NamedCommands.registerCommand("GroundIntakeCycle", 
    //     Commands.sequence(
    //         // 1. Pivot Down
    //         Commands.run(m_pivot::pivotDownAuto, m_pivot)
    //             .until(() -> m_pivot.getPivotPosition() >= Constants.PIVOT_DOWN_POSITION),
            
    //         // 2. Run Intake UNTIL note is found OR time runs out
    //         Commands.run(m_pivot::IntakeOn, m_pivot)
    //             .until(m_pivot::hasNote),  // <--- THE KEY FIX: Stops the moment sensor triggers
    //             // .withTimeout(3.0),        // <--- SAFETY NET: Stops if we miss the ball after 3s

    //         // 3. Force Stop Intake (Crucial to ensure it doesn't keep coasting)
    //         Commands.runOnce(m_pivot::IntakeOff, m_pivot),

    //         // 4. Pivot Up
    //         Commands.run(m_pivot::pivotUpAuto, m_pivot)
    //             .until(() -> m_pivot.getPivotPosition() <= Constants.PIVOT_UP_POSITION),

    //         Commands.run(m_pivot::IntakeOn, m_pivot)
    //           .withTimeout(0.5)
    //     )
    // );
    //     // --- REGISTER NAMED COMMANDS FOR PATHPLANNER ---
    //     // m_leds.setIdleRainbow();
    //   }

    }
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        // joystick.povRight().onTrue(new AlignLimelight(true, drivetrain).withTimeout(3));



        // groundIntakeButton.whileTrue(new InstantCommand(m_pivot :: pivotDown)).onFalse(new InstantCommand(m_pivot::off));
        

        m_controller2.axisGreaterThan(1, 0.7).whileTrue(new RunCommand(m_pivot::setPivotDown)).onFalse(new InstantCommand(m_pivot::off)); 
        m_controller2.axisLessThan(1, -0.7).whileTrue(new RunCommand(m_pivot::setPivotUp)).onFalse(Commands.parallel(new InstantCommand(m_pivot::off))); 
        //InBelt
        m_controller2.rightBumper().onTrue(new InstantCommand(m_belt:: jammed)). onFalse(new InstantCommand(m_belt:: off));
        m_controller2.axisGreaterThan(3, .7).onTrue(new InstantCommand(m_belt:: intake)).onFalse(new InstantCommand(m_belt:: off));
        
        //Shooter 
        // m_controller2.axisGreaterThan(2, .7).whileTrue(new ParallelCommandGroup(new InstantCommand(m_shooter:: shoot1), new InstantCommand(m_Led:: setGreen))).onFalse(new ParallelCommandGroup(new InstantCommand(m_shooter::off), new InstantCommand(m_pivot::off)));
        m_controller2.axisGreaterThan(2, .7).onTrue(new InstantCommand(m_shooter::shoot1)).onFalse(new InstantCommand(m_shooter::off));
        m_controller2.leftBumper().whileTrue(new ParallelCommandGroup(new InstantCommand(m_shooter::shoot1), beltCommand(), new InstantCommand(m_Led:: setPink))).onFalse(new ParallelCommandGroup( new InstantCommand(m_shooter::off), new InstantCommand(m_belt::off), new InstantCommand(m_Led:: setGreen)));
        
        //Hood
        raiseHood.whileTrue(new RunCommand(m_hood:: HoodUp)).onFalse(new InstantCommand(m_hood:: HoodOff));
        lowerHood.whileTrue(new RunCommand(m_hood:: HoodDown)).onFalse(new InstantCommand(m_hood:: HoodOff));
        //new RunCommand(m_pivot::IntakeOn, m_pivot)
          // new RunCommand(m_leds:: setIntaking, m_leds)
          // "Up" Intake Button

        //Limelight
        AutoAlign.whileTrue(m_limelight.alignToTargetCommand());
    }

    public Command getAutonomousCommand() {

        // An example command will be run in autonomous
        // return Commands.sequence(new WaitCommand(0.25), resetOdometry, myTrajectory);
        // return new HardcodedAuton(m_drive, m_pivot, m_elevator, m_shintake);
        System.out.println("getAutonomousCommand");
        return new PathPlannerAuto("polina");
    
        // // Simple drive forward auton
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     drivetrain.applyRequest(() -> idle)
        // );
    }
}
