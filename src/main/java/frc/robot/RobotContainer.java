// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.auto.AutoActions;
import frc.robot.commands.CoralIntakeAssistCommand;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.commands.aimSequences.NetAimCommand;
import frc.robot.commands.aimSequences.ReefAimCommand;
import frc.robot.commands.aimSequences.SuperCycleCommand;
import frc.robot.subsystems.beambreak.BeambreakIOReal;
import frc.robot.subsystems.beambreak.BeambreakIOSim;
import frc.robot.subsystems.climber.ClimberIOReal;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.indicator.IndicatorIO;
import frc.robot.subsystems.indicator.IndicatorIOARGB;
import frc.robot.subsystems.indicator.IndicatorIOSim;
import frc.robot.subsystems.indicator.IndicatorSubsystem;
import frc.robot.subsystems.limelight.LimelightIOReal;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.photonvision.PhotonVisionIOReal;
import frc.robot.subsystems.photonvision.PhotonVisionIOSim;
import frc.robot.subsystems.photonvision.PhotonVisionSubsystem;
import frc.robot.subsystems.questnav.QuestNavSubsystem;
import frc.robot.subsystems.roller.RollerIOReal;
import frc.robot.subsystems.roller.RollerIOSim;
import frc.robot.subsystems.superstructure.DestinationSupplier;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.SuperstructureState;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOReal;
import frc.robot.subsystems.superstructure.elevator.ElevatorIOSim;
import frc.robot.subsystems.superstructure.elevator.ElevatorSubsystem;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmPivotIOReal;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmPivotIOSim;
import frc.robot.subsystems.superstructure.endeffectorarm.EndEffectorArmSubsystem;
import frc.robot.subsystems.superstructure.intake.IntakePivotIOReal;
import frc.robot.subsystems.superstructure.intake.IntakePivotIOSim;
import frc.robot.subsystems.superstructure.intake.IntakeSubsystem;
import frc.robot.utils.BlocklessEitherCommand;
import lib.ironpulse.rbd.TransformRecorder;
import lib.ironpulse.swerve.Swerve;
import lib.ironpulse.swerve.SwerveCommands;
import lib.ironpulse.swerve.sim.ImuIOSim;
import lib.ironpulse.swerve.sim.SwerveModuleIOSimpleSim;
import lib.ironpulse.swerve.sjtu6.ImuIOPigeon;
import lib.ironpulse.swerve.sjtu6.SwerveModuleIOSJTU6;
import lib.ironpulse.utils.TimeDelayedBoolean;
import lombok.Getter;
import org.littletonrobotics.AllianceFlipUtil;

import java.util.HashMap;

import static edu.wpi.first.units.Units.*;
import static frc.robot.RobotConstants.LimelightConstants.LIMELIGHT_LEFT;
import static frc.robot.RobotConstants.LimelightConstants.LIMELIGHT_RIGHT;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandGenericHID streamDeckController = new CommandGenericHID(1);
  private final CommandXboxController testerController = new CommandXboxController(2);
  // Update Manager
  private final DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();
  // @Getter
  // private final LoggedDashboardChooser<String> autoChooser;
  // private final AutoActions autoActions;
  // private final AutoFile autoFile;
  // Subsystems
  private Swerve swerve;
  private ElevatorSubsystem elevatorSubsystem;
  private IntakeSubsystem intakeSubsystem;
  @Getter
  private ClimberSubsystem climberSubsystem;
  private IndicatorSubsystem indicatorSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private EndEffectorArmSubsystem endEffectorArmSubsystem;
  private Superstructure superstructure;
  private QuestNavSubsystem questNavSubsystem;
  private PhotonVisionSubsystem photonVisionSubsystem;
  private RobotStateRecorder robotStateRecorder = RobotStateRecorder.getInstance(); // NOTE: better to init beforehead
  private double lastResetTime = 0.0;
  private TimeDelayedBoolean netEjectTimer = new TimeDelayedBoolean(RobotConstants.EndEffectorArmConstants.NET_SHOOT_DELAY_TIME.get());
  private TimeDelayedBoolean processorEjectTimer = new TimeDelayedBoolean(RobotConstants.EndEffectorArmConstants.PROCESSOR_SHOOT_DELAY_TIME.get());
  private TimeDelayedBoolean l1ShootSideEjectTimer = new TimeDelayedBoolean(RobotConstants.EndEffectorArmConstants.L1_SHOOT_SIDE_EJECT_DELAY_TIME.get());

  public RobotContainer() {
    if (RobotBase.isReal()) {
      // Real hardware initialization
      swerve = new Swerve(
          RobotConstants.SwerveConstants.kRealConfig,
          new ImuIOPigeon(RobotConstants.SwerveConstants.kRealConfig),
          new SwerveModuleIOSJTU6(RobotConstants.SwerveConstants.kRealConfig, 0),
          new SwerveModuleIOSJTU6(RobotConstants.SwerveConstants.kRealConfig, 1),
          new SwerveModuleIOSJTU6(RobotConstants.SwerveConstants.kRealConfig, 2),
          new SwerveModuleIOSJTU6(RobotConstants.SwerveConstants.kRealConfig, 3));
      indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOARGB());
      elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
      intakeSubsystem = new IntakeSubsystem(
          new IntakePivotIOReal(),
          new RollerIOReal(
              RobotConstants.IntakeConstants.INTAKE_MOTOR_ID,
              RobotConstants.CANIVORE_CAN_BUS_NAME,
              60,
              60,
              RobotConstants.IntakeConstants.IS_INTAKER_INVERT,
              RobotConstants.IntakeConstants.IS_BRAKE),
          new RollerIOReal(
              RobotConstants.IntakeConstants.INDEX_MOTOR_ID,
              RobotConstants.CANIVORE_CAN_BUS_NAME,
              RobotConstants.IntakeConstants.STATOR_CURRENT_LIMIT_AMPS,
              RobotConstants.IntakeConstants.SUPPLY_CURRENT_LIMIT_AMPS,
              RobotConstants.IntakeConstants.IS_INDEXER_INVERT,
              RobotConstants.IntakeConstants.IS_BRAKE),
          new BeambreakIOReal(RobotConstants.BeamBreakConstants.INTAKE_BEAMBREAK_ID));
      climberSubsystem = new ClimberSubsystem(new ClimberIOReal());
      endEffectorArmSubsystem = new EndEffectorArmSubsystem(
          new EndEffectorArmPivotIOReal(),
          new RollerIOReal(
              RobotConstants.EndEffectorArmConstants.END_EFFECTOR_ARM_ROLLER_MOTOR_ID,
              RobotConstants.CANIVORE_CAN_BUS_NAME,
              RobotConstants.EndEffectorArmConstants.STATOR_CURRENT_LIMIT_AMPS,
              RobotConstants.EndEffectorArmConstants.SUPPLY_CURRENT_LIMIT_AMPS,
              RobotConstants.EndEffectorArmConstants.IS_INVERT,
              RobotConstants.EndEffectorArmConstants.IS_BRAKE),
          new BeambreakIOReal(RobotConstants.BeamBreakConstants.ENDEFFECTORARM_CORAL_BEAMBREAK_ID),
          new BeambreakIOReal(RobotConstants.BeamBreakConstants.ENDEFFECTORARM_ALGAE_BEAMBREAK_ID));
      limelightSubsystem = new LimelightSubsystem(new HashMap<>() {
        {
          put(LIMELIGHT_LEFT, new LimelightIOReal(LIMELIGHT_LEFT));
          put(LIMELIGHT_RIGHT, new LimelightIOReal(LIMELIGHT_RIGHT));
        }
      });
      photonVisionSubsystem = new PhotonVisionSubsystem(new PhotonVisionIOReal(0));
    } else {
      // Simulation initialization
      swerve = new Swerve(
          RobotConstants.SwerveConstants.kSimConfig,
          new ImuIOSim(),
          new SwerveModuleIOSimpleSim(RobotConstants.SwerveConstants.kSimConfig, 0),
          new SwerveModuleIOSimpleSim(RobotConstants.SwerveConstants.kSimConfig, 1),
          new SwerveModuleIOSimpleSim(RobotConstants.SwerveConstants.kSimConfig, 2),
          new SwerveModuleIOSimpleSim(RobotConstants.SwerveConstants.kSimConfig, 3));

      indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOSim());
      elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOSim());
      intakeSubsystem = new IntakeSubsystem(
          new IntakePivotIOSim(),
          new RollerIOSim(1, RobotConstants.IntakeConstants.ROLLER_RATIO,
              new SimpleMotorFeedforward(0.0, 0.24),
              new ProfiledPIDController(0.5, 0.0, 0.0,
                  new TrapezoidProfile.Constraints(15, 1))),
          new RollerIOSim(1, 1.0, new SimpleMotorFeedforward(0.0, 0.24),
              new ProfiledPIDController(0.5, 0.0, 0.0,
                  new TrapezoidProfile.Constraints(15, 1))),
          new BeambreakIOSim(RobotConstants.BeamBreakConstants.INTAKE_BEAMBREAK_ID));
      climberSubsystem = new ClimberSubsystem(new ClimberIOSim());
      limelightSubsystem = new LimelightSubsystem(new HashMap<>() {
        {
          put(LIMELIGHT_LEFT, new LimelightIOReal(LIMELIGHT_LEFT));
          put(LIMELIGHT_RIGHT, new LimelightIOReal(LIMELIGHT_RIGHT));
        }
      });
      endEffectorArmSubsystem = new EndEffectorArmSubsystem(
          new EndEffectorArmPivotIOSim(),
          new RollerIOSim(1, 1.0, new SimpleMotorFeedforward(0.0, 0.24),
              new ProfiledPIDController(0.5, 0.0, 0.0,
                  new TrapezoidProfile.Constraints(15, 1))),
          new BeambreakIOSim(RobotConstants.BeamBreakConstants.ENDEFFECTORARM_CORAL_BEAMBREAK_ID),
          new BeambreakIOSim(RobotConstants.BeamBreakConstants.ENDEFFECTORARM_ALGAE_BEAMBREAK_ID));
      photonVisionSubsystem = new PhotonVisionSubsystem(new PhotonVisionIOSim(0));
    }
    superstructure = new Superstructure(intakeSubsystem, endEffectorArmSubsystem, elevatorSubsystem);

    AutoActions.init(swerve, superstructure, indicatorSubsystem, photonVisionSubsystem);

    configureDriverBindings();
    configureStreamDeckBindings();
    configureTesterBindings();
    configureOthers();
  }

  private void configureDriverBindings() {
    swerve.setDefaultCommand(
        SwerveCommands.driveWithJoystick(
            swerve,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            RobotStateRecorder::getPoseDriverRobotCurrent,
            MetersPerSecond.of(0.04),
            DegreesPerSecond.of(3.0)));

    driverController.start().onTrue(
        swerve.defer(() -> {
          return
              SwerveCommands.resetAngle(swerve, AllianceFlipUtil.apply(Rotation2d.k180deg))
              .alongWith(
                  Commands.runOnce(() -> {
                    RobotStateRecorder.getInstance().resetTransform(
                        TransformRecorder.kFrameWorld,
                        TransformRecorder.kFrameRobot);
                    indicatorSubsystem.indicateWithTimeout(IndicatorIO.Patterns.RESET_ODOM, 0.5).schedule();
                  }))
              .ignoringDisable(true);
        }).ignoringDisable(true)
    );

    // INTAKE and OUTTAKE
    driverController
        .rightStick()
        .toggleOnTrue(
            Commands.parallel(
                    new CoralIntakeAssistCommand(
                        swerve,
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        RobotStateRecorder::getPoseDriverRobotCurrent,
                        MetersPerSecond.of(0.04),
                        DegreesPerSecond.of(3.0)
                    ),
                    superstructure.runGoal(this::determineIntakeState)
                ).until(this::isIntakeComplete)
                .andThen(() -> {
                      indicatorSubsystem.indicateWithTimeout(IndicatorIO.Patterns.AFTER_INTAKE, 0.5).schedule();
                    }
                )
        );

    driverController.b().whileTrue(superstructure.runGoal(() -> SuperstructureState.CORAL_OUTTAKE));
    driverController.y().toggleOnTrue(superstructure.runGoal(() -> SuperstructureState.SAFE_OUTTAKE));
    driverController.a().onTrue(superstructure.toggleIntakePose());

    //CLIMBER
    driverController.povDown().whileTrue(
        Commands.either(
            Commands.run(() ->
                climberSubsystem.setWantedState(ClimberSubsystem.WantedState.IDLE)),
            Commands.run(() ->
                climberSubsystem.setWantedState(ClimberSubsystem.WantedState.DEPLOY)),
            climberSubsystem::hasDeployed
        ));

    //SCORING
    driverController
        .leftBumper()
        .whileTrue(
            new BlocklessEitherCommand(
                // algae
                Commands.parallel(
                        Commands.deadline(
                            new NetAimCommand(swerve, indicatorSubsystem, () -> driverController.getLeftX() * 4.5),
                            AutoActions.applySwerveLimit()
                        ),
                        Commands.waitUntil(() -> {
                          Pose2d poseWorldRobot = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
                          return AimGoalSupplier.isNearNet(poseWorldRobot);
                        }).andThen(
                            superstructure.runGoal(() -> SuperstructureState.NET_SCORE).until(superstructure::poseAtGoal)
                        ).andThen(
                            Commands.waitUntil(() -> {
                              return RobotStateRecorder.getVelocityWorldRobotCurrent().getTranslation().getNorm() < 0.20;
                            })
                        )
                    )
                    .andThen(
                        Commands.runOnce(() -> netEjectTimer.reset())
                            .andThen(superstructure
                                .runGoal(() -> SuperstructureState.NET_SCORE_EJECT)
                                .until(() -> netEjectTimer.update(!superstructure.hasAlgae())))),
                // coral
                createScoringCommand(false, SuperstructureState.L4),
                superstructure::hasAlgae));
    driverController
        .leftTrigger()
        .whileTrue(
            new BlocklessEitherCommand(
                // processor
                Commands.deadline(
                        Commands.waitUntil(() -> driverController.rightTrigger().getAsBoolean()),
                        superstructure.runGoal(() -> SuperstructureState.PROCESSOR_SCORE))
                    .andThen(
                        Commands.runOnce(() -> processorEjectTimer.reset())
                            .andThen(superstructure.runGoal(() -> SuperstructureState.PROCESSOR_SCORE_EJECT)
                                .until(() -> processorEjectTimer.update(!superstructure.hasAlgae())))
                    ),
                new BlocklessEitherCommand(
                    createScoringCommand(false, SuperstructureState.L3),
                    superstructure.runGoal(
                        () -> driverController.rightTrigger().getAsBoolean() ? SuperstructureState.L1_INTAKE_SIDE_EJECT : SuperstructureState.L1_INTAKE_SIDE
                    ),
                    () -> superstructure.hasCoral() || superstructure.getState() == SuperstructureState.CORAL_GROUND_INTAKE || superstructure.getState() == SuperstructureState.CORAL_INDEXED_INTAKE
                ),
                superstructure::hasAlgae
            ));
    driverController
        .back()
        .whileTrue(
            createScoringCommand(false, SuperstructureState.L2));
    driverController
        .rightBumper()
        .whileTrue(
            createScoringCommand(true, SuperstructureState.L4));
    driverController
        .rightTrigger()
        .whileTrue(
            createScoringCommand(true, SuperstructureState.L3)
        );
    driverController
        .leftStick()
        .whileTrue(
            createScoringCommand(true, SuperstructureState.L2));

    driverController.povUp().whileTrue(
        superstructure.runGoal(SuperstructureState.CORAL_L1_INTAKE)
    );


    //TESTING : TODO: remove
    //     driverController.x().whileTrue(
//     Commands.runOnce(() -> {
//     destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.CORAL_SCORING);
//     })
//     .andThen(
//     new ReefAimCommand(swerve, indicatorSubsystem)
//     )
//     );
//    driverController.x().whileTrue(
//        Commands.deadline(
//            AutoActions.chase().until(AutoActions::isInIntakeDangerZone),
//            Commands.run(
//                () -> {
//                  if (AutoActions.isCoralInSight()) {
//                    indicatorSubsystem.setPattern(IndicatorIO.Patterns.ASSISTED_INTAKE);
//                  } else {
//                    indicatorSubsystem.setPattern(IndicatorIO.Patterns.INTAKE);
//                  }
//                }, indicatorSubsystem)
//        ).finallyDo(
//            () -> {
//              indicatorSubsystem.indicateWithTimeout(IndicatorIO.Patterns.AFTER_INTAKE, 0.6).schedule();
//            }
//        )
//    );
    driverController.x().onTrue(
        superstructure.runZero()
    );
  }

  private void configureStreamDeckBindings() {

  }

  public void configureTesterBindings() {
    testerController.a().whileTrue(superstructure.runGoal(() -> SuperstructureState.CORAL_INDEXED_INTAKE));
    testerController.y().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.L4)
            .until(testerController.rightTrigger())
            .andThen(
                superstructure.runGoal(() -> SuperstructureState.L4_EJECT)
                    .until(() -> !superstructure.hasCoral())));
    testerController.x().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.L3)
            .until(testerController.rightTrigger())
            .andThen(
                superstructure.runGoal(() -> SuperstructureState.L3_EJECT)
                    .until(() -> !superstructure.hasCoral())));
    testerController.b().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.L2)
            .until(testerController.rightTrigger())
            .andThen(
                superstructure.runGoal(() -> SuperstructureState.L2_EJECT)
                    .until(() -> !superstructure.hasCoral())));
    testerController.povUp().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.P2)
            .until(() -> superstructure.hasAlgae()));
    testerController.povDown().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.P1)
            .until(() -> superstructure.hasAlgae()));
    testerController.leftBumper().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.NET_SCORE)
            .until(testerController.rightTrigger())
            .andThen(
                Commands.runOnce(() -> netEjectTimer.reset())
                    .andThen(superstructure.runGoal(() -> SuperstructureState.NET_SCORE_EJECT)
                        .until(() -> netEjectTimer.update(!superstructure.hasAlgae())))));
    testerController.rightBumper().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.L1_SHOOT_SIDE)
            .until(testerController.rightTrigger())
            .andThen(
                Commands.runOnce(() -> l1ShootSideEjectTimer.reset())
                    .andThen(superstructure.runGoal(() -> SuperstructureState.L1_SHOOT_SIDE_EJECT)
                        .until(() -> l1ShootSideEjectTimer.update(!superstructure.hasCoral())))));
    testerController.start().whileTrue(
        Commands.runOnce(() -> {
              destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.CORAL_SCORING);
            })
            .andThen(
                new ReefAimCommand(swerve, indicatorSubsystem)));
    testerController.back().onTrue(superstructure.toggleIntakePose());

  }

  public void configureOthers() {
    indicatorSubsystem.setDefaultCommand(
        Commands.runOnce(() -> {
          if (!DriverStation.isEnabled()) {
            if(DriverStation.isDSAttached()) {
              indicatorSubsystem.setPattern(
                  AllianceFlipUtil.shouldFlip() ? IndicatorIO.Patterns.RED_ALLIANCE : IndicatorIO.Patterns.BLUE_ALLIANCE
              );
            } else {
              indicatorSubsystem.setPattern(
                  IndicatorIO.Patterns.LOSS
              );
            }
          } else {
            // highest priority: climb
            if (climberSubsystem.getSystemState() == ClimberSubsystem.SystemState.CLIMBING) {
              indicatorSubsystem.setPattern(IndicatorIO.Patterns.CLIMB_FINISHED);
            } else if (climberSubsystem.getSystemState() == ClimberSubsystem.SystemState.DEPLOYING) {
              indicatorSubsystem.setPattern(IndicatorIO.Patterns.CLIMB_DEPLOYED);
            } else {
              // second prioritye: superstructure
              if (superstructure.getState() == SuperstructureState.CORAL_OUTTAKE
                  || superstructure.getState() == SuperstructureState.SAFE_OUTTAKE
              ) {
                indicatorSubsystem.setPattern(IndicatorIO.Patterns.INTAKE);
              } else {
                // third priority: in edge case
                if(AimGoalSupplier.isEdgeCase(RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d()))
                  indicatorSubsystem.setPattern(IndicatorIO.Patterns.EDGE_CASE);
                else
                  indicatorSubsystem.setPattern(IndicatorIO.Patterns.NORMAL);
              }
            }
          }
        }, indicatorSubsystem).onlyIf(() -> !indicatorSubsystem.isOutsideDefault()).ignoringDisable(true)
    );
  }

  public FieldConstants.AprilTagLayoutType getAprilTagLayoutType() {
    return FieldConstants.aprilTagType;
  }

  public void setMegaTag2(boolean setMegaTag2) {
    limelightSubsystem.setMegaTag2(setMegaTag2);
  }

  /**
   * Helper method to create a scoring command sequence for a given branch and
   * state.
   * Logs the branch and state for debugging and analysis.
   *
   * @param isRightBranch true for right branch, false for left branch
   * @param state         the superstructure state to target
   * @return the command sequence for scoring
   */
  private Command createScoringCommand(boolean isRightBranch, SuperstructureState state) {
    return new BlocklessEitherCommand(
        // If we have coral, run the SuperCycle command
        Commands.sequence(
            Commands.runOnce(() -> {
              destinationSupplier.updateBranch(isRightBranch);
            }),
            Commands.runOnce(() -> {
              destinationSupplier.setStateSetPoint(state);
            }),
            new SuperCycleCommand(swerve, superstructure, indicatorSubsystem, true)
        ),
        // If no coral, check if intake button (right stick) is pressed
        new BlocklessEitherCommand(
            // If intake button is pressed, run reef aim command then supercycle when coral is intaken
            Commands.runOnce(() -> {
                  destinationSupplier.updateBranch(isRightBranch);
                  destinationSupplier.setStateSetPoint(state);
                  destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.CORAL_SCORING);
                }).andThen(
                    Commands.parallel(
                        superstructure.runGoal(() -> SuperstructureState.CORAL_GROUND_INTAKE),
                        new ReefAimCommand(swerve, indicatorSubsystem)
                    ).until(superstructure::hasCoral)
                )
                .andThen(new SuperCycleCommand(swerve, superstructure, indicatorSubsystem, true)),
            // If intake button is not pressed, do nothing
            Commands.none(),
            () -> superstructure.getState() == SuperstructureState.CORAL_GROUND_INTAKE
        ),
        () -> superstructure.hasCoral()
    );
  }

  /**
   * Helper method to check if robot is in the hexagonal reef danger zone
   *
   * @return true if robot is in danger zone
   */
  private boolean isInReefDangerZone() {
    Pose2d pose = RobotStateRecorder.getPoseWorldRobotCurrent().toPose2d();
    return AimGoalSupplier.isInHexagonalReefDangerZone(pose);
  }

  /**
   * Determines the appropriate intake state based on current conditions
   *
   * @return SuperstructureState for intake operation
   */
  private SuperstructureState determineIntakeState() {
    boolean hasAlgae = superstructure.hasAlgae();
    boolean inDangerZone = isInReefDangerZone();

    // If we have algae OR we're in danger zone, use indexed intake
    // Otherwise, use ground intake for safety
    if (hasAlgae || inDangerZone) {
      return SuperstructureState.CORAL_INDEXED_INTAKE;
    } else {
      return SuperstructureState.CORAL_GROUND_INTAKE;
    }
  }

  /**
   * Determines if the intake operation is complete based on current conditions
   *
   * @return true if intake is complete
   */
  private boolean isIntakeComplete() {
    boolean hasAlgae = superstructure.hasAlgae();
    boolean inDangerZone = isInReefDangerZone();

    if (hasAlgae) {
      // When we have algae, we need both algae AND indexed coral
      return superstructure.hasAlgae() && superstructure.hasIndexedCoral();
    } else if (!inDangerZone) {
      // When not in danger zone, we just need coral
      return superstructure.hasCoral();
    } else {
      // When in danger zone (without algae), we need indexed coral
      return superstructure.hasIndexedCoral();
    }
  }

  /**
   * Creates a command to move the superstructure to algae prestate until robot
   * exits danger zone
   * This is used after SuperCycleCommand completes to ensure safe exit from reef
   * area
   *
   * @return Command that continues to algae prestate until out of danger zone
   */
  private Command createDangerZoneExitCommand() {
    return Commands.sequence(
            // Set game piece to algae intaking to get correct prestate
            Commands.runOnce(() -> destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.ALGAE_INTAKING)),
            // Continue running to algae prestate until out of danger zone
            superstructure
                .runGoal(() -> destinationSupplier.getPreState())
                .until(() -> !isInReefDangerZone()))
        .onlyIf(this::isInReefDangerZone); // Only run if actually in danger zone
  }

  public void robotPeriodic() {
    limelightSubsystem.estimatedPose.ifPresent(estimate -> {
      if (estimate[0] != null) {
        swerve.addVisionMeasurement(
            new Pose3d(estimate[0].pose()), estimate[0].timestampSeconds(), VecBuilder.fill(0.1, 0.1, 0.3, 100.0));
      }

      if (estimate[1] != null) {
        swerve.addVisionMeasurement(
            new Pose3d(estimate[1].pose()), estimate[1].timestampSeconds(), VecBuilder.fill(0.1, 0.1, 0.3, 100.0));
      }
    });

    var now = Seconds.of(Timer.getTimestamp());
    RobotStateRecorder.getInstance().putTransform(
        swerve.getEstimatedPose(), now,
        TransformRecorder.kFrameWorld,
        TransformRecorder.kFrameRobot);
    RobotStateRecorder.putVelocityRobot(now, swerve.getChassisSpeeds());


    RobotStateRecorder.periodic();
  }
}
