// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.commands.aimSequences.SuperCycleCommand;
import frc.robot.commands.climb.ClimbCommand;
import frc.robot.commands.climb.IdleClimbCommand;
import frc.robot.commands.climb.PreClimbCommand;
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
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.ElevatorIO;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOReal;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffectorarm.*;
import frc.robot.subsystems.superstructure.intake.*;
import frc.robot.subsystems.questnav.OculusIOReal;
import frc.robot.subsystems.questnav.OculusSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.roller.RollerIO;
import frc.robot.subsystems.roller.RollerIOReal;
import frc.robot.subsystems.roller.RollerIOSim;
import lombok.Getter;
import org.frcteam6941.looper.UpdateManager;
import org.littletonrobotics.AllianceFlipUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import frc.robot.subsystems.questnav.OculusIO;

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
    private OculusSubsystem oculusSubsystem;
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
                        // new RollerIOReal(
                        //     RobotConstants.IntakeConstants.INDEX_MOTOR_ID,
                        //     RobotConstants.CANIVORE_CAN_BUS_NAME,
                        //     RobotConstants.IntakeConstants.STATOR_CURRENT_LIMIT_AMPS,
                        //     RobotConstants.IntakeConstants.SUPPLY_CURRENT_LIMIT_AMPS,
                        //     RobotConstants.IntakeConstants.IS_INVERT,
                        //     RobotConstants.IntakeConstants.IS_BRAKE
                        // ),
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
                oculusSubsystem = new OculusSubsystem(new OculusIOReal());
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
                        // new RollerIOSim(1, 1.0, new SimpleMotorFeedforward(0.0, 0.24),
                        //         new ProfiledPIDController(0.5, 0.0, 0.0,
                        //                 new TrapezoidProfile.Constraints(15, 1))),
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
                oculusSubsystem = new OculusSubsystem(new OculusIOReal());
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
                    // new RollerIO() {
                    // },
                    new BeambreakIO() {
                    }
            );
        }
        if (elevatorSubsystem == null) {
            elevatorSubsystem = new ElevatorSubsystem(new ElevatorIO() {
            });
        }
        if (oculusSubsystem == null) {
            oculusSubsystem = new OculusSubsystem(new OculusIO.Default());
        }


        superstructure = new Superstructure(intakeSubsystem, endEffectorArmSubsystem, elevatorSubsystem);

        // Initialize the update manager
        updateManager = new UpdateManager(swerve,
                display);
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


    private void configureDriverBindings() {
        //TODO: consider enabling the auto scoring button whilst the superstructure is intaking
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
        // driverController
        //         .povUp()
        //         .whileTrue(
        //             Commands.parallel(
        //                     superstructure
        //                             .runGoal(() -> SuperstructureState.AVOID),
        //                     new PreClimbCommand(climberSubsystem)
        //             )
        //         );

        // driverController
        //         .y()
        //         .whileTrue(
        //                 new ClimbCommand(climberSubsystem)
        //                         .onlyIf(
        //                                 () -> superstructure.getState() == SuperstructureState.AVOID
        //                         )
        //         );

        // driverController
        //         .povLeft()
        //         .whileTrue(
        //                 Commands.parallel(
        //                         superstructure
        //                                 .runGoal(() -> SuperstructureState.AVOID),
        //                         new IdleClimbCommand(climberSubsystem)
        //                 )
        //         );




        driverController
                .a()
                .whileTrue(
                        superstructure
                                .runGoal(() -> {
                                    if (superstructure.hasAlgae()) {
                                        return SuperstructureState.CORAL_INDEXED_INTAKE;
                                    } else if (!AimGoalSupplier.isInHexagonalReefDangerZone(
                                            swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()))) {
                                        return SuperstructureState.CORAL_GROUND_INTAKE;
                                    } else {
                                        return SuperstructureState.CORAL_INDEXED_INTAKE;
                                    }
                                })
                                .until(() -> {
                                    if (superstructure.hasAlgae()) {
                                        return superstructure.hasAlgae() && superstructure.indexedCoral();
                                    } else if (!AimGoalSupplier.isInHexagonalReefDangerZone(
                                            swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()))) {
                                        return superstructure.hasCoral();
                                    } else {
                                        return superstructure.indexedCoral();
                                    }
                                })
                );
        driverController
                .x()
                .whileTrue(
                        superstructure
                                .runGoal(() -> SuperstructureState.NET_SCORE)
                                .until(driverController.y())
                                .andThen(
                                        superstructure
                                                .runGoal(() -> SuperstructureState.NET_SCORE_EJECT)
                                                .until(() -> !superstructure.hasAlgae())
                                )
                );

        // Left trigger binding - only executes if there is coral
        driverController
                .leftBumper()
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> destinationSupplier.updateBranch(false)),
                               Commands.runOnce(() -> destinationSupplier.setStateSetPoint(SuperstructureState.L4)),
                               new SuperCycleCommand(superstructure,
                                indicatorSubsystem,
                                driverController,
                                () -> false)
                        )
                        .onlyIf(() -> superstructure.hasCoral())
                );
        driverController
                .leftTrigger()
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> destinationSupplier.updateBranch(false)),
                               Commands.runOnce(() -> destinationSupplier.setStateSetPoint(SuperstructureState.L3)),
                               new SuperCycleCommand(superstructure,
                                indicatorSubsystem,
                                driverController,
                                () -> false)
                        )
                        .onlyIf(() -> superstructure.hasCoral())
                );
        driverController
                .back()
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> destinationSupplier.updateBranch(false)),
                               Commands.runOnce(() -> destinationSupplier.setStateSetPoint(SuperstructureState.L2)),
                               new SuperCycleCommand(superstructure,
                                indicatorSubsystem,
                                driverController,
                                () -> false)
                        )
                        .onlyIf(() -> superstructure.hasCoral())
                );
        driverController
                .rightBumper()
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> destinationSupplier.updateBranch(true)),
                               Commands.runOnce(() -> destinationSupplier.setStateSetPoint(SuperstructureState.L4)),
                               new SuperCycleCommand(superstructure,
                                indicatorSubsystem,
                                driverController,
                                () -> false)
                        )
                        .onlyIf(() -> superstructure.hasCoral())
                );
        driverController
                .rightTrigger()
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> destinationSupplier.updateBranch(true)),
                               Commands.runOnce(() -> destinationSupplier.setStateSetPoint(SuperstructureState.L3)),
                               new SuperCycleCommand(superstructure,
                                indicatorSubsystem,
                                driverController,
                                () -> false)
                        )
                        .onlyIf(() -> superstructure.hasCoral())
                );
        driverController
                .leftStick()
                .whileTrue(
                        Commands.sequence(
                                Commands.runOnce(() -> destinationSupplier.updateBranch(true)),
                               Commands.runOnce(() -> destinationSupplier.setStateSetPoint(SuperstructureState.L2)),
                               new SuperCycleCommand(superstructure,
                                indicatorSubsystem,
                                driverController,
                                () -> false)
                        )
                        .onlyIf(() -> superstructure.hasCoral())
                );
        
    }

    private void configureStreamDeckBindings() {
        streamDeckController
                .button(1)
                .onTrue(
                        Commands.runOnce(
                                () -> DestinationSupplier
                                        .getInstance()
                                        .setStateSetPoint(SuperstructureState.L4)
                        )
                );

        streamDeckController
                .button(2)
                .onTrue(
                        Commands.runOnce(
                                () -> DestinationSupplier
                                        .getInstance()
                                        .setStateSetPoint(SuperstructureState.L3)
                        )
                );
    }

    public void configureTesterBindings() {


        testerController
                .b()
                .whileTrue(
                        superstructure
                                .runGoal(() -> SuperstructureState.L4)
                                .until(testerController.x())
                                .andThen(
                                        superstructure
                                                .runGoal(() -> SuperstructureState.L4_EJECT)
                                                .until(() -> !superstructure.hasCoral())
                                )
                );

        testerController
                .x()
                .whileTrue(
                        superstructure
                                .runGoal(() -> SuperstructureState.P1)
                                .until(superstructure::hasAlgae)
                );

        testerController
                .leftBumper()
                .whileTrue(
                        superstructure
                                .runGoal(() -> SuperstructureState.NET_SCORE)
                                .until(testerController.rightBumper())
                                .andThen(
                                        superstructure
                                                .runGoal(() -> SuperstructureState.NET_SCORE_EJECT)
                                                .until(() -> !superstructure.hasAlgae())
                                )
                )
        ;
    }

    public Command getAutonomousCommand() {
        // return autoFile.runAuto(autoChooser.get());
        return Commands.none();
    }

    public FieldConstants.AprilTagLayoutType getAprilTagLayoutType() {
        return FieldConstants.defaultAprilTagType;
    }

    public void setMegaTag2(boolean setMegaTag2) {
        limelightSubsystem.setMegaTag2(setMegaTag2);
    }

}
