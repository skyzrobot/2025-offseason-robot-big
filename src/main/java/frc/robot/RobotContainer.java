// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.display.Display;
import frc.robot.subsystems.beambreak.BeambreakIO;
import frc.robot.subsystems.beambreak.BeambreakIOReal;
import frc.robot.subsystems.beambreak.BeambreakIOSim;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorIOARGB;
import frc.robot.subsystems.indicator.IndicatorIOSim;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.limelight.LimelightIOReal;
import frc.robot.subsystems.limelight.LimelightIOReplay;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import frc.robot.subsystems.superstructure.GamepieceTracker;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOReal;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffectorarm.*;
import frc.robot.subsystems.superstructure.intake.*;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOReal;
import frc.robot.subsystems.roller.RollerIOSim;
import lombok.Getter;
import org.frcteam6941.looper.UpdateManager;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import java.util.HashMap;

import static frc.robot.RobotConstants.LimelightConstants.LIMELIGHT_LEFT;
import static frc.robot.RobotConstants.LimelightConstants.LIMELIGHT_RIGHT;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {


    // Flag states
    public static boolean elevatorIsDanger;
    public static boolean intakeHasCoral = false;
    public static boolean endeffectorIsDanger = false;
    public static boolean overrideEndEffectorDanger = false;

    // Controllers
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandGenericHID streamDeckController = new CommandGenericHID(1);
    private final CommandXboxController testerController = new CommandXboxController(2);
    // Update Manager
    @Getter
    private final UpdateManager updateManager;
    private final Display display = Display.getInstance();
    private final DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();
    // @Getter
    // private final LoggedDashboardChooser<String> autoChooser;
    // private final AutoActions autoActions;
    // private final AutoFile autoFile;
    // Subsystems
    private final Swerve swerve = Swerve.getInstance();
    private ElevatorSubsystem elevatorSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ClimberSubsystem climberSubsystem;
    private IndicatorSubsystem indicatorSubsystem;
    private LimelightSubsystem limelightSubsystem;
    private EndEffectorArmSubsystem endEffectorArmSubsystem;
    private Superstructure superstructure;
    private double lastResetTime = 0.0;


    public RobotContainer() {
        if (!RobotConstants.useReplay) {
            if (RobotBase.isReal()) {
                // Real hardware initialization
                indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOARGB());
                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
                intakeSubsystem = new IntakeSubsystem(
                        new IntakePivotIOReal(),
                        new RollerIOReal(
                            RobotConstants.IntakeConstants.INTAKE_MOTOR_ID,
                            RobotConstants.CANIVORE_CAN_BUS_NAME,
                            RobotConstants.IntakeConstants.STATOR_CURRENT_LIMIT_AMPS,
                            RobotConstants.IntakeConstants.SUPPLY_CURRENT_LIMIT_AMPS,
                            RobotConstants.IntakeConstants.IS_INVERT,
                            RobotConstants.IntakeConstants.IS_BRAKE
                        ),
                        new BeambreakIOReal(RobotConstants.BeamBreakConstants.INTAKE_BEAMBREAK_ID)
                );
                climberSubsystem = new ClimberSubsystem(new ClimberIOReal());
                endEffectorArmSubsystem = new EndEffectorArmSubsystem(
                        new EndEffectorArmPivotIOReal(),
                        new RollerIOReal(
                            RobotConstants.EndEffectorArmConstants.END_EFFECTOR_ARM_ROLLER_MOTOR_ID,
                            RobotConstants.CANIVORE_CAN_BUS_NAME,
                            RobotConstants.EndEffectorArmConstants.STATOR_CURRENT_LIMIT_AMPS,
                            RobotConstants.EndEffectorArmConstants.SUPPLY_CURRENT_LIMIT_AMPS,
                            RobotConstants.EndEffectorArmConstants.IS_INVERT,
                            RobotConstants.EndEffectorArmConstants.IS_BRAKE
                        ),
                        new BeambreakIOReal(RobotConstants.BeamBreakConstants.ENDEFFECTORARM_CORAL_BEAMBREAK_ID),
                        new BeambreakIOReal(RobotConstants.BeamBreakConstants.ENDEFFECTORARM_ALGAE_BEAMBREAK_ID)
                );
                limelightSubsystem = new LimelightSubsystem(new HashMap<>() {{
                    put(LIMELIGHT_LEFT, new LimelightIOReal(LIMELIGHT_LEFT));
                    put(LIMELIGHT_RIGHT, new LimelightIOReal(LIMELIGHT_RIGHT));
                }});
            } else {
                // Simulation initialization
                indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOSim());
                elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
                intakeSubsystem = new IntakeSubsystem(
                        new IntakePivotIOSim(),
                        new RollerIOSim(1, RobotConstants.IntakeConstants.ROLLER_RATIO, 
                            new SimpleMotorFeedforward(0.0, 0.24),
                            new ProfiledPIDController(0.5, 0.0, 0.0,
                                new TrapezoidProfile.Constraints(15, 1))),
                        new BeambreakIOSim(RobotConstants.BeamBreakConstants.INTAKE_BEAMBREAK_ID)
                );
                climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
                limelightSubsystem = new LimelightSubsystem(new HashMap<>() {{
                    put(LIMELIGHT_LEFT, new LimelightIOReal(LIMELIGHT_LEFT));
                    put(LIMELIGHT_RIGHT, new LimelightIOReal(LIMELIGHT_RIGHT));
                }});
                endEffectorArmSubsystem = new EndEffectorArmSubsystem(
                        new EndEffectorArmPivotIOSim(),
                        new RollerIOSim(1, 1.0, new SimpleMotorFeedforward(0.0, 0.24),
                                new ProfiledPIDController(0.5, 0.0, 0.0,
                                        new TrapezoidProfile.Constraints(15, 1))),
                        new BeambreakIOSim(RobotConstants.BeamBreakConstants.ENDEFFECTORARM_CORAL_BEAMBREAK_ID),
                        new BeambreakIOSim(RobotConstants.BeamBreakConstants.ENDEFFECTORARM_ALGAE_BEAMBREAK_ID)
                );
            }
        }

        // in the case of replay, no implementation is needed
        if (indicatorSubsystem == null) {
            indicatorSubsystem = new IndicatorSubsystem(new IndicatorIO() {
            });
        }
        if (limelightSubsystem == null) {
            limelightSubsystem = new LimelightSubsystem(new HashMap<>() {{
                put(LIMELIGHT_LEFT, new LimelightIOReplay("LimelightL"));
                put(LIMELIGHT_RIGHT, new LimelightIOReplay("LimelightR"));
            }});
        }
        if (endEffectorArmSubsystem == null) {
            endEffectorArmSubsystem = new EndEffectorArmSubsystem(
                    new EndEffectorArmPivotIO() {
                    },
                    new RollerIO() {
                    },
                    new BeambreakIO() {
                    },
                    new BeambreakIO() {
                    }
            );
        }
        if (climberSubsystem == null) {
            climberSubsystem = new ClimberSubsystem(new ClimberIO() {
            });
        }
        if (intakeSubsystem == null) {
            intakeSubsystem = new IntakeSubsystem(
                    new IntakePivotIO() {
                    },
                    new RollerIO() {
                    },
                    new BeambreakIO() {
                    }
            );
        }
        if (elevatorSubsystem == null) {
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {
            });
        }

        superstructure = new Superstructure(intakeSubsystem,endEffectorArmSubsystem,elevatorSubsystem);

        // Initialize the update manager
        updateManager = new UpdateManager(swerve,
                display, destinationSupplier);
        updateManager.registerAll();

        //autoChooser = new LoggedDashboardChooser<>("Chooser", CustomAutoChooser.buildAutoChooser("New Auto"));
        //autoActions = new AutoActions(indicatorSubsystem, elevatorSubsystem, endEffectorArmSubsystem, intakeSubsystem);
        //autoFile = new AutoFile(autoActions);


        //configureAutoChooser();
        configureDriverBindings();
        configureStreamDeckBindings();
        configureTesterBindings();
    }

    // private void configureAutoChooser() {
    //     autoChooser.addOption("4CoralLeft", "4CoralLeft");
    //     autoChooser.addOption("4CoralRight", "4CoralRight");
    //     autoChooser.addOption("1Coral1AlgaeMiddle", "1Coral1AlgaeMiddle");
    //     autoChooser.addOption("1Coral3AlgaeMiddle", "1Coral3AlgaeMiddle");
    //     autoChooser.addOption("Test", "Test");
    //     autoChooser.addOption("None", "None");
    // }


    
/**
 * Superstructure Default Command Behavior
 * 
 * The superstructure subsystem uses a default command that continuously monitors and updates its state.
 * This command runs automatically in the background under these conditions:
 * - When the robot transitions from disabled to enabled state
 * - When no other command is actively running on the subsystem
 * - After a previously running command completes
 * 
 * Note: When a goal command is executed (e.g., via button bindings), it remains active even after
 * reaching the target state. This prevents the default command from taking over, which is intentional
 * to maintain the desired state until a new command is issued.
 */
    private void configureDriverBindings() {
        swerve.setDefaultCommand(Commands.runOnce(() -> swerve.drive(
                new Translation2d(
                        Math.abs(driverController.getLeftY()) < RobotConstants.SwerveConstants.deadband ?
                                0 : -driverController.getLeftY() * RobotConstants.SwerveConstants.maxSpeed.magnitude(),
                        Math.abs(driverController.getLeftX()) < RobotConstants.SwerveConstants.deadband ?
                                0 : -driverController.getLeftX() * RobotConstants.SwerveConstants.maxSpeed.magnitude()),
                Math.abs(-driverController.getRightX()) < RobotConstants.SwerveConstants.rotationalDeadband ?
                        0 : -driverController.getRightX() * RobotConstants.SwerveConstants.maxAngularRate.magnitude(),
                true,
                false), swerve));

        driverController.start().onTrue(
                Commands.runOnce(() -> {
                    /*
                        TODO: the reset command will be activated twice when the start button is pressed only once,
                        this is only a temporary solution to avoid execute the command twice within 0.01s,
                        please fix the bug
                    */
                    if (Timer.getFPGATimestamp() - lastResetTime > 0.01) {
                        swerve.resetHeadingController();
                        swerve.resetPose(
                                AllianceFlipUtil.apply(new Pose2d(
                                        new Translation2d(0, 0),
                                        swerve.getLocalizer().getLatestPose().getRotation())));
                    }
                    lastResetTime = Timer.getFPGATimestamp();
                    indicatorSubsystem.setPattern(IndicatorIO.Patterns.RESET_ODOM);
                }).ignoringDisable(true));
        // // Climbing
        // driverController.povUp().whileTrue(new PreClimbCommand(climberSubsystem, elevatorSubsystem, intakeSubsystem, endEffectorArmSubsystem));
        // driverController.povLeft().whileTrue(new IdleClimbCommand(climberSubsystem, elevatorSubsystem, intakeSubsystem, endEffectorArmSubsystem));
        // driverController.y().whileTrue(new ClimbCommand(climberSubsystem, elevatorSubsystem, intakeSubsystem, endEffectorArmSubsystem));
        // // Intake and outtake
        // driverController.leftTrigger().toggleOnTrue(switchIntakeModeCommand());
        // driverController.b().toggleOnTrue(new GroundOuttakeCommand(intakeSubsystem));
        // driverController.leftBumper().whileTrue(
        //         Commands.sequence(
        //                 Commands.runOnce(() -> elevatorSubsystem.setElevatorPosition(destinationSupplier.getElevatorSetpoint(false))),
        //                 Commands.waitUntil(() -> elevatorSubsystem.elevatorReady(0.03)),
        //                 Commands.runOnce(() -> endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.ALGAE_INTAKE)),
        //                 Commands.waitSeconds(100)
        //         ).finallyDo(() -> {
        //             if (!GamepieceTracker.getInstance().isEndeffectorHasCoral() && !GamepieceTracker.getInstance().isEndeffectorHasAlgae()) {
        //                 elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.HOME_EXTENSION_METERS.get());
        //             } else {
        //                 elevatorSubsystem.setElevatorPosition(RobotConstants.ElevatorConstants.HOLD_EXTENSION_METERS.get());
        //             }
        //             endEffectorArmSubsystem.setWantedState(EndEffectorArmSubsystem.WantedState.HOLD);
        //         }));
        // // Scoring
        // driverController.povRight().whileTrue(switchAimingModeCommand());
        // driverController.rightBumper().whileTrue(switchPreMoveModeCommand());
        // driverController.leftStick().whileTrue(switchAimingModeCommand().beforeStarting(Commands.runOnce(() -> destinationSupplier.updateBranch(false))).ignoringDisable(true));
        // driverController.rightStick().whileTrue(switchAimingModeCommand().beforeStarting(Commands.runOnce(() -> destinationSupplier.updateBranch(true))).ignoringDisable(true));
        // driverController.povDown().onTrue(new ZeroElevatorCommand(elevatorSubsystem, intakeSubsystem, endEffectorArmSubsystem));

        // //Intake Stuck Fix
        // driverController.back().whileTrue(new FixIntakeCommand(elevatorSubsystem, intakeSubsystem, endEffectorArmSubsystem));
    }

    private void configureStreamDeckBindings() {
        // // Operator's controller
        // streamDeckController.button(1).onTrue(Commands.runOnce(() -> destinationSupplier.setCurrentControlMode(DestinationSupplier.controlMode.MANUAL)).ignoringDisable(true));
        // streamDeckController.button(3).onTrue(Commands.runOnce(() -> destinationSupplier.setCurrentControlMode(DestinationSupplier.controlMode.AUTO)).ignoringDisable(true));
        // streamDeckController.button(5).onTrue(Commands.runOnce(destinationSupplier::switchUseSuperCycle).ignoringDisable(true));
        // streamDeckController.button(6).onTrue(Commands.runOnce(() -> destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.CORAL_SCORING)).ignoringDisable(true));
        // streamDeckController.button(7).onTrue(Commands.runOnce(() -> destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.ALGAE_INTAKING)).ignoringDisable(true));
        // streamDeckController.button(11).onTrue(Commands.runOnce(() -> destinationSupplier.setAlgaeScoringMode(DestinationSupplier.AlgaeScoringMode.NET)).ignoringDisable(true));
        // streamDeckController.button(12).onTrue(Commands.runOnce(() -> destinationSupplier.setAlgaeScoringMode(DestinationSupplier.AlgaeScoringMode.PROCESSOR)).ignoringDisable(true));
        // streamDeckController.button(14).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L2)).ignoringDisable(true));
        // streamDeckController.button(15).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L3)).ignoringDisable(true));
        // streamDeckController.button(16).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.L4)).ignoringDisable(true));
        // streamDeckController.button(18).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.P1)).ignoringDisable(true));
        // streamDeckController.button(19).onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.P2)).ignoringDisable(true));
        // streamDeckController.button(8).whileTrue(Commands.run(() -> destinationSupplier.setCurrentL1Mode(DestinationSupplier.L1Mode.INTAKE))
        //         .finallyDo(() -> destinationSupplier.setCurrentL1Mode(DestinationSupplier.L1Mode.ELEVATOR)).ignoringDisable(true));
        // streamDeckController.button(10).whileTrue(Commands.run(() -> intakeSubsystem.setLowerAngle(true))).onFalse(Commands.runOnce(() -> intakeSubsystem.setLowerAngle(false)));
    }

    public void configureTesterBindings() {
        testerController.a().onTrue(superstructure.runGoal(() -> SuperstructureState.CORAL_STOW));
        testerController.b().onTrue(superstructure.runGoal(() -> SuperstructureState.L3));
        testerController.x().toggleOnTrue(superstructure.runGoal(() -> SuperstructureState.L1_SHOOT_SIDE_EJECT));
        testerController.y().onTrue(superstructure.runGoal(() -> SuperstructureState.CORAL_GROUND_INTAKE));
        testerController.leftBumper().onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.P1)));
        testerController.rightBumper().onTrue(Commands.runOnce(() -> destinationSupplier.updateElevatorSetpoint(DestinationSupplier.elevatorSetpoint.P2)));
    }

    // public Command getAutonomousCommand() {
    //     return autoFile.runAuto(autoChooser.get());
    // }

    public FieldConstants.AprilTagLayoutType getAprilTagLayoutType() {
        return FieldConstants.defaultAprilTagType;
    }

    // public Command switchAimingModeCommand() {
    //     return new ConditionalCommand(
    //             // AUTO
    //             new AutoAimShootCommand(
    //                     indicatorSubsystem, endEffectorArmSubsystem, elevatorSubsystem, intakeSubsystem,
    //                     () -> false, driverController),
    //             // MANUAL
    //             Commands.sequence(
    //                     Commands.runOnce(() -> DestinationSupplier.getInstance().setCurrentGamePiece(DestinationSupplier.GamePiece.CORAL_SCORING)),
    //                     new ReefAimCommand(() -> false, elevatorSubsystem, driverController, indicatorSubsystem)),
    //             () -> destinationSupplier.getCurrentControlMode() == DestinationSupplier.controlMode.AUTO);
    // }

    // public Command switchIntakeModeCommand() {
    //     return new ConditionalCommand(
    //             new GroundIntakeCommand(indicatorSubsystem, intakeSubsystem, endEffectorArmSubsystem, elevatorSubsystem),
    //             new HoldIntakeCommand(indicatorSubsystem, intakeSubsystem, elevatorSubsystem),
    //             () -> destinationSupplier.getL1Mode() == DestinationSupplier.L1Mode.ELEVATOR || endEffectorArmSubsystem.hasAlgae());
    // }

    // public Command switchPreMoveModeCommand() {
    //     return new ConditionalCommand(
    //             // Intake L1
    //             new ShootHoldCommand(intakeSubsystem, () -> driverController.rightTrigger().getAsBoolean()),
    //             new ConditionalCommand(
    //                     new ConditionalCommand(
    //                             // Elevator Algae Net
    //                             new PutAlgaeNetCommand(driverController, endEffectorArmSubsystem, elevatorSubsystem, intakeSubsystem, indicatorSubsystem),
    //                             // Elevator Algae Processor
    //                             new PutAlgaeProcessorCommand(driverController, endEffectorArmSubsystem, elevatorSubsystem, intakeSubsystem, indicatorSubsystem),
    //                             () -> destinationSupplier.getAlgaeScoringMode() == DestinationSupplier.AlgaeScoringMode.NET
    //                     ),
    //                     // Elevator Coral
    //                     new PutCoralCommand(driverController, endEffectorArmSubsystem, elevatorSubsystem, intakeSubsystem, indicatorSubsystem),
    //                     () -> !GamepieceTracker.getInstance().isEndeffectorHasCoral()
    //             ),
    //             () -> destinationSupplier.getL1Mode() == DestinationSupplier.L1Mode.INTAKE);
    // }

    public void setMegaTag2(boolean setMegaTag2) {
        limelightSubsystem.setMegaTag2(setMegaTag2);
    }

}
