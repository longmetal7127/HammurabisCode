// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.HootEpilogueBackend;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Indexer.IndexerSetpoint;
import frc.robot.subsystems.Lintake;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Spindexer.SpindexerSetpoint;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Flywheel.FlywheelSetpoint;
import frc.robot.subsystems.Hood;
import frc.robot.util.CommandGamesirController;
import frc.robot.util.FuelSim;
import frc.robot.vision.LoggableRobotPose;
import frc.robot.vision.PhotonVisionSystem;

@Logged
public class Robot extends TimedRobot {

    private enum IntakeMode {
        /** Mode A: on release, move lintake to 0.1m and stop rollers. */
        A,
        /** Mode B: on release, just stop rollers. */
        B
    }

    private IntakeMode intakeMode = IntakeMode.A;

    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                        // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandGamesirController joystick = new frc.robot.util.CommandGamesirController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final PhotonVisionSystem vision = new PhotonVisionSystem(this::consumePhotonVisionMeasurement,
            () -> drivetrain.getState().Pose);

    private Command m_autonomousCommand;
    public Lintake lintake = new Lintake();
    public Turret turret = new Turret();
    public Hood hood = new Hood();
    public Indexer indexer = new Indexer();
    public Spindexer spindexer = new Spindexer();
    public Flywheel flywheel = new Flywheel();
    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {
        DogLog.setOptions(new DogLogOptions().withCaptureNt(true));
        Epilogue.configure(config -> {
            // Log to both the Phoenix 6 SignalLogger
            // and NT4 backends
            config.backend = EpilogueBackend.multi(
                    new HootEpilogueBackend(),
                    new NTEpilogueBackend(NetworkTableInstance.getDefault()));

            if (Utils.isSimulation()) {
                // Re-throw any errors that occur in simulation
                config.errorHandler = ErrorHandler.crashOnError();
            }

            // ...
        });
        Epilogue.bind(this);

        CommandScheduler.getInstance().onCommandInitialize(CommandsLogging::commandStarted);
        CommandScheduler.getInstance().onCommandFinish(CommandsLogging::commandEnded);
        CommandScheduler.getInstance()
                .onCommandInterrupt(
                        (interrupted, interrupting) -> {
                            interrupting.ifPresent(
                                    interrupter -> CommandsLogging.runningInterrupters.put(interrupter, interrupted));
                            CommandsLogging.commandEnded(interrupted);
                        });

        configureBindings();
        if (Robot.isSimulation()) {
            FuelSim.getInstance().spawnStartingFuel();
            FuelSim.getInstance().registerRobot(
                    Units.inchesToMeters(25.5), // from left to right
                    Units.inchesToMeters(29), // from front to back
                    Units.inchesToMeters(6),
                    () -> drivetrain.getPose(),
                    () -> drivetrain.getFieldRelativeSpeeds());

            FuelSim.getInstance().registerIntake(
                    Units.inchesToMeters(17.475),
                    Units.inchesToMeters(27.117),
                    Units.inchesToMeters(-12.725),
                    Units.inchesToMeters(-2.725), // robot-centric coordinates for bounding box in meters
                    lintake::getIntakeEnabled // (optional) BooleanSupplier for whether the intake should be active at a
            // given moment
            );

        }
        FuelSim.getInstance().start();
        URCL.start(DataLogManager.getLog());

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                   // negative Y
                                                                                                   // (forward)
                        .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.leftTrigger().whileTrue(
                Commands.print("Shoot stub: left trigger held"));

        joystick.rightTrigger().whileTrue(
                Commands.parallel(
                        lintake.setHeightCommand(lintake.getMaxHeightMeters()),
                        lintake.setVelocityCommand(1.0)))
                .onFalse(
                        Commands.either(
                                Commands.parallel(
                                        lintake.setHeightCommand(0.1),
                                        lintake.stopCommand()),
                                lintake.stopCommand(),
                                () -> intakeMode == IntakeMode.A));

        joystick.leftBumper().onTrue(
                Commands.runOnce(() -> intakeMode = IntakeMode.A));

        joystick.rightBumper().onTrue(
                Commands.runOnce(() -> intakeMode = IntakeMode.B));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public void consumePhotonVisionMeasurement(LoggableRobotPose pose) {
        drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                        .withVelocityY(0)
                        .withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        CommandsLogging.logRunningCommands();
        CommandsLogging.logRequiredSubsystems();

        logger.updateMechanismPoses(lintake.getMechanismPose(),
                turret.getMechanismPose(),
                hood.getMechanismPose(turret.getMechanismPose().getRotation()));

    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
        FuelSim.getInstance().updateSim();

    }
}
