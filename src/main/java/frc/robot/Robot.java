// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.HootAutoReplay;
import com.ctre.phoenix6.HootEpilogueBackend;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.logging.EpilogueBackend;
import edu.wpi.first.epilogue.logging.NTEpilogueBackend;
import edu.wpi.first.epilogue.logging.errors.ErrorHandler;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.Shooter.ShooterMode;
import frc.robot.subsystems.Lintake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Spindexer.SpindexerSetpoint;
import frc.robot.util.CommandGamesirController;
import frc.robot.util.FuelSim;
import frc.robot.vision.CameraConfig;
import frc.robot.vision.LoggableRobotPose;
import frc.robot.vision.PhotonVisionSystem;

@Logged
public class Robot extends TimedRobot {

        public enum IntakeMode {
                /** Mode A: on release, move lintake to 0.1m and stop rollers. */
                A,
                /** Mode B: on release, just stop rollers. */
                B
        }

        private IntakeMode intakeMode = IntakeMode.A;

        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
        private double MaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandGamesirController joystick = new frc.robot.util.CommandGamesirController(0);
        private final CommandGamesirController operatorJoystick = new frc.robot.util.CommandGamesirController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        /*
         * Four cameras: back-left, back-right, left-side, right-side.
         * TODO: Replace the placeholder Transform3d values below with the real
         * measured offsets from robot-center to each camera.
         * Translation3d(X forward, Y left, Z up) in meters,
         * Rotation3d(roll, pitch, yaw) in radians.
         */
        public final PhotonVisionSystem vision = new PhotonVisionSystem(
                        this::consumePhotonVisionMeasurement,
                        () -> drivetrain.getState().Pose,
                        new CameraConfig("back-left", new Transform3d(
                                        new Translation3d(Inches.of(-12.895), Inches.of(10.918), Inches.of(9.224)),
                                        new Rotation3d(0, Math.toRadians(-55), Math.toRadians(150)))),
                        new CameraConfig("back-right", new Transform3d(
                                        new Translation3d(Inches.of(-12.895), Inches.of(-10.918), Inches.of(9.224)),
                                        new Rotation3d(0, Math.toRadians(-55), Math.toRadians(-150)))),
                        new CameraConfig("left-side", new Transform3d(
                                        new Translation3d(Inches.of(1.105), Inches.of(12.257), Inches.of(8.235)),
                                        new Rotation3d(0, Math.toRadians(-35), Math.toRadians(90)))),
                        new CameraConfig("right-side", new Transform3d(
                                        new Translation3d(Inches.of(-4.187), Inches.of(-10.888), Inches.of(10.735)),
                                        new Rotation3d(Degrees.of(0), Degrees.of(-35), Degrees.of(-90))

                        )));

        private Command m_autonomousCommand;
        public Lintake lintake = new Lintake();
        public Indexer indexer = new Indexer();
        public Spindexer spindexer = new Spindexer();
        public Shooter shooter;
        public final FuelSim fuelSim = new FuelSim();

        /* log and replay timestamp and joystick data */
        private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
                        .withTimestampReplay()
                        .withJoystickReplay();
        private final AutoFactory autoFactory;
        private final AutoRoutines autoRoutines;
        private final AutoChooser autoChooser = new AutoChooser();

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
                                                                        interrupter -> CommandsLogging.runningInterrupters
                                                                                        .put(interrupter, interrupted));
                                                        CommandsLogging.commandEnded(interrupted);
                                                });

                // Initialize shooter after drivetrain so we can pass suppliers
                shooter = new Shooter(
                                () -> drivetrain.getPose(),
                                () -> drivetrain.getFieldRelativeSpeeds(),
                                fuelSim);

                configureBindings();

                if (Robot.isSimulation()) {
                        fuelSim.spawnStartingFuel();
                        fuelSim.registerRobot(
                                        Inches.of(25.5), // from left to right
                                        Inches.of(29), // from front to back
                                        Inches.of(6),
                                        () -> drivetrain.getPose(),
                                        () -> drivetrain.getFieldRelativeSpeeds());

                        fuelSim.registerIntake(
                                        Inches.of(17.475),
                                        Inches.of(27.117),
                                        Inches.of(-12.725),
                                        Inches.of(-2.725), // robot-centric coordinates for bounding box
                                        lintake::getIntakeEnabled // (optional) BooleanSupplier for whether the intake
                                                                  // should be active
                        );

                }
                fuelSim.start();
                URCL.start(DataLogManager.getLog());
                autoFactory = drivetrain.createAutoFactory();
                autoRoutines = new AutoRoutines(autoFactory);

                autoChooser.addRoutine("SimplePath", autoRoutines::simplePathAuto);
                SmartDashboard.putData("Auto Chooser", autoChooser);

        }

        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
                drivetrain.setDefaultCommand(
                                // Drivetrain will execute this command periodically
                                drivetrain.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive
                                                                                                                   // forward
                                                                                                                   // with
                                                                                                                   // negative
                                                                                                                   // Y
                                                                                                                   // (forward)
                                                .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with
                                                                                                // negative X (left)
                                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive
                                                                                                            // counterclockwise
                                                                                                            // with
                                                                                                            // negative
                                                                                                            // X (left)
                                ));
                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();

                RobotModeTriggers.disabled().onTrue(
                                drivetrain.playMusic().ignoringDisable(true));
                RobotModeTriggers.teleop().onTrue(
                                drivetrain.stopMusic());
                joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
                /*
                 * RobotModeTriggers.disabled().whileTrue(
                 * drivetrain.applyRequest(() -> idle).ignoringDisable(true));
                 */

                // --- Shoot (left trigger) ---
                // While held: aim turret + hood, spin flywheel, feed indexer/spindexer
                joystick.rightTrigger().whileTrue(
                                shooter.buildShootCommand(
                                                spindexer.setTargetTemporary(SpindexerSetpoint.Spin),
                                                indexer.setTargetTemporary(IndexerSetpoint.Index), ShooterMode.AgainstHub))
                                .onFalse(
                                                shooter.setHoodAngleCommand(0));

                joystick.leftTrigger().whileTrue(
                                Commands.sequence(
                                                lintake.setHeightCommand(lintake.getMaxHeightMeters()),
                                                lintake.setVelocityCommand(0.6)))
                                .onFalse(
                                                Commands.either(
                                                                Commands.sequence(
                                                                                lintake.setHeightCommand(0.1),
                                                                                lintake.stopCommand()),
                                                                lintake.stopCommand(),
                                                                () -> intakeMode == IntakeMode.A));

                joystick.leftBumper().onTrue(
                                Commands.runOnce(() -> intakeMode = IntakeMode.A));

                joystick.rightBumper().onTrue(
                                Commands.runOnce(() -> intakeMode = IntakeMode.B));

                // --- Tuning: while A is held, use DogLog tunables as absolute setpoints ---
                operatorJoystick.povUp().whileTrue(
                 shooter.buildTuningCommand());

                                 operatorJoystick.povDown().whileTrue(
                 spindexer.setTargetTemporary(SpindexerSetpoint.Spin).alongWith(
                                                indexer.setTargetTemporary(IndexerSetpoint.Index)));


                // turret autoaiadd m stuff
                joystick.x().whileTrue(
                                shooter.buildShootCommand(
                                                spindexer.setTargetTemporary(SpindexerSetpoint.Spin),
                                                indexer.setTargetTemporary(IndexerSetpoint.Index), ShooterMode.Autoaim))
                                .onFalse(
                                                shooter.setHoodAngleCommand(0));
                joystick.y().whileTrue(
                                shooter.buildShootCommand(
                                                spindexer.setTargetTemporary(SpindexerSetpoint.Spin),
                                                indexer.setTargetTemporary(IndexerSetpoint.Index), ShooterMode.Pass))
                                .onFalse(
                                                shooter.setHoodAngleCommand(0));
                operatorJoystick.b().onTrue(lintake.zeroingRoutine());
                operatorJoystick.a().whileTrue(
                        spindexer.setTargetTemporaryBck(SpindexerSetpoint.SpinReverse).alongWith(
                        indexer.setTargetTemporary(IndexerSetpoint.SpinReverse)));
                operatorJoystick.y().whileTrue(
                        shooter.runReverse().alongWith(
                        lintake.runReverse())).onFalse(
                                shooter.resetFlywheel().alongWith(
                                lintake.stopCommand()));
                operatorJoystick.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
                
                //sysid
                // joystick.x().onTrue(drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
                // joystick.y().onTrue(drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward));

                drivetrain.registerTelemetry(logger::telemeterize);

        }

        public void consumePhotonVisionMeasurement(LoggableRobotPose pose) {
                drivetrain.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
        }

        public Command getAutonomousCommand() {
                // Simple drive forward auton
                final var idle = new SwerveRequest.Idle();
                return Commands.sequence(
                                Commands.waitSeconds(0.5),
                                // shoot as expected
                                shooter.buildShootCommand(
                                                spindexer.setTargetTemporary(SpindexerSetpoint.Spin),
                                                indexer.setTargetTemporary(IndexerSetpoint.Index), ShooterMode.AgainstHub)
                                                .withTimeout(5),
                                lintake.setHeightCommand(lintake.getMaxHeightMeters()),
                                Commands.waitSeconds(1.5),
                                lintake.setVelocityCommand(-0.1),
                                Commands.waitSeconds(1.5),
                                lintake.stopCommand(),
                                shooter.buildShootCommand(
                                                spindexer.setTargetTemporary(SpindexerSetpoint.Spin),
                                                indexer.setTargetTemporary(IndexerSetpoint.Index), ShooterMode.AgainstHub)
                                                .withTimeout(5),
                                /*
                                 * drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                                 * // Then slowly drive forward (away from us) for 5 seconds.
                                 * drivetrain.applyRequest(() -> drive.withVelocityX(0.5)
                                 * .withVelocityY(0)
                                 * .withRotationalRate(0))
                                 * .withTimeout(5.0),
                                 */

                                // Reset our field centric heading to match the robot
                                // facing away from our alliance station wall (0 deg).
                                shooter.setHoodAngleCommand(0),
                                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                                // Finally idle for the rest of auton
                                autoChooser.selectedCommand(),

                                drivetrain.applyRequest(() -> idle));

        }

        @Override
        public void robotPeriodic() {
                m_timeAndJoystickReplay.update();
                CommandScheduler.getInstance().run();
                CommandsLogging.logRunningCommands();
                CommandsLogging.logRequiredSubsystems();
                vision.periodic();
                logger.updateMechanismPoses(lintake.getMechanismPose(),
                                shooter.getTurretMechanismPose(),
                                shooter.getHoodMechanismPose());

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
                fuelSim.updateSim();

        }
}
