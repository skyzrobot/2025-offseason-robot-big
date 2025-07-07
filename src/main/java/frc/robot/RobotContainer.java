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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.CoralIntakeAssistCommand;
import frc.robot.commands.aimSequences.*;
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
import lib.ironpulse.swerve.sim.SwerveModuleIOSim;
import lib.ironpulse.swerve.sjtu6.ImuIOPigeon;
import lib.ironpulse.swerve.sjtu6.SwerveModuleIOSJTU6;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;

import static edu.wpi.first.units.Units.*;
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
  private final DestinationSupplier destinationSupplier = DestinationSupplier.getInstance();
  // @Getter
  // private final LoggedDashboardChooser<String> autoChooser;
  // private final AutoActions autoActions;
  // private final AutoFile autoFile;
  // Subsystems
  private Swerve swerve;
  private ElevatorSubsystem elevatorSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private ClimberSubsystem climberSubsystem;
  private IndicatorSubsystem indicatorSubsystem;
  private LimelightSubsystem limelightSubsystem;
  private EndEffectorArmSubsystem endEffectorArmSubsystem;
  private Superstructure superstructure;
  private QuestNavSubsystem questNavSubsystem;
  private PhotonVisionSubsystem photonVisionSubsystem;
  private RobotStateRecorder robotStateRecorder = RobotStateRecorder.getInstance(); // NOTE: better to init beforehead
  private double lastResetTime = 0.0;


  public RobotContainer() {
    if (RobotBase.isReal()) {
      // Real hardware initialization
      swerve = new Swerve(
          RobotConstants.SwerveConstants.kRealConfig,
          new ImuIOPigeon(RobotConstants.SwerveConstants.kRealConfig),
          new SwerveModuleIOSJTU6(RobotConstants.SwerveConstants.kRealConfig, 0),
          new SwerveModuleIOSJTU6(RobotConstants.SwerveConstants.kRealConfig, 1),
          new SwerveModuleIOSJTU6(RobotConstants.SwerveConstants.kRealConfig, 2),
          new SwerveModuleIOSJTU6(RobotConstants.SwerveConstants.kRealConfig, 3)
      );
      indicatorSubsystem = new IndicatorSubsystem(new IndicatorIOARGB());
      elevatorSubsystem = new ElevatorSubsystem(new ElevatorIOReal());
      intakeSubsystem = new IntakeSubsystem(
          new IntakePivotIOReal(),
          new RollerIOReal(
              RobotConstants.IntakeConstants.INTAKE_MOTOR_ID,
              RobotConstants.CANIVORE_CAN_BUS_NAME,
              RobotConstants.IntakeConstants.STATOR_CURRENT_LIMIT_AMPS,
              RobotConstants.IntakeConstants.SUPPLY_CURRENT_LIMIT_AMPS,
              RobotConstants.IntakeConstants.IS_INTAKER_INVERT,
              RobotConstants.IntakeConstants.IS_BRAKE
          ),
          new RollerIOReal(
              RobotConstants.IntakeConstants.INDEX_MOTOR_ID,
              RobotConstants.CANIVORE_CAN_BUS_NAME,
              RobotConstants.IntakeConstants.STATOR_CURRENT_LIMIT_AMPS,
              RobotConstants.IntakeConstants.SUPPLY_CURRENT_LIMIT_AMPS,
              RobotConstants.IntakeConstants.IS_INDEXER_INVERT,
              RobotConstants.IntakeConstants.IS_BRAKE,
              RobotConstants.IntakeConstants.INDEX_FOLLOWER_MOTOR_ID,
              RobotConstants.IntakeConstants.INDEX_FOLLOWER_INVERT
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
      photonVisionSubsystem = new PhotonVisionSubsystem(new PhotonVisionIOReal(0));
    } else {
      // Simulation initialization
      swerve = new Swerve(
          RobotConstants.SwerveConstants.kSimConfig,
          new ImuIOSim(),
          new SwerveModuleIOSim(RobotConstants.SwerveConstants.kSimConfig, 0),
          new SwerveModuleIOSim(RobotConstants.SwerveConstants.kSimConfig, 1),
          new SwerveModuleIOSim(RobotConstants.SwerveConstants.kSimConfig, 2),
          new SwerveModuleIOSim(RobotConstants.SwerveConstants.kSimConfig, 3)
      );

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
      photonVisionSubsystem = new PhotonVisionSubsystem(new PhotonVisionIOSim(0));
    }


    superstructure = new Superstructure(intakeSubsystem, endEffectorArmSubsystem, elevatorSubsystem);

    //autoChooser = new LoggedDashboardChooser<>("Chooser", CustomAutoChooser.buildAutoChooser("New Auto"));
    //autoActions = new AutoActions(indicatorSubsystem, elevatorSubsystem, endEffectorArmSubsystem, intakeSubsystem);
    //autoFile = new AutoFile(autoActions);

//    CommandScheduler.getInstance().unregisterAllSubsystems();
//    CommandScheduler.getInstance().registerSubsystem(
//        climberSubsystem, swerve
//    );
    CommandScheduler.getInstance().unregisterSubsystem(climberSubsystem);

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
    swerve.setDefaultCommand(
        SwerveCommands.driveWithJoystick(
            swerve,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            RobotStateRecorder::getPoseDriverRobotCurrent,
            MetersPerSecond.of(0.04),
            DegreesPerSecond.of(3.0)
        )
    );


    driverController.start().onTrue(SwerveCommands.reset(
        swerve, new Pose3d(
            new Pose2d(new Translation2d(0, 0), new Rotation2d()))
    ).alongWith(Commands.runOnce(
        () -> {
          RobotStateRecorder.getInstance().resetTransform(
              TransformRecorder.kFrameWorld,
              TransformRecorder.kFrameRobot
          );
          lastResetTime = Timer.getFPGATimestamp();
          indicatorSubsystem.setPattern(IndicatorIO.Patterns.RESET_ODOM);
        })));


    // Coral intake - chooses between ground and indexed intake based on conditions
    driverController
        .a()
        .toggleOnTrue(
            superstructure
                .runGoal(this::determineIntakeState)
                .until(this::isIntakeComplete)
        );

    driverController.b().whileTrue(superstructure.runGoal(() -> SuperstructureState.CORAL_OUTTAKE));
//    driverController.x().whileTrue(
//        Commands.runOnce(() -> {
//              destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.CORAL_SCORING);
//            })
//            .andThen(
//                new ReefAimCommand(swerve, indicatorSubsystem)
//            )
//    );
//    driverController.x().whileTrue(
//        new ChaseCoralCommand(swerve, photonVisionSubsystem)
//    );

    driverController.x().whileTrue(
        Commands.either(
            Commands.run(() -> climberSubsystem.setWantedState(ClimberSubsystem.WantedState.CLIMB)),
            Commands.run(() -> climberSubsystem.setWantedState(ClimberSubsystem.WantedState.DEPLOY)),
            climberSubsystem::hasDeployed
        )
    );

    // Y button - Coral intake assist drive
    driverController.y().whileTrue(
        new CoralIntakeAssistCommand(
            swerve,
            photonVisionSubsystem,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX(),
            RobotStateRecorder::getPoseDriverRobotCurrent,
            MetersPerSecond.of(0.04),
            DegreesPerSecond.of(3.0)
        )
    );

    // Left trigger binding - only executes if there is coral
    driverController
        .leftBumper()
        .whileTrue(
            new BlocklessEitherCommand(
                createScoringCommand(false, SuperstructureState.L4),
                new NetAimCommand(swerve, () -> driverController.getLeftX() * 2.5)
                    .alongWith(
                        superstructure.runGoal(() -> SuperstructureState.NET_SCORE)
                            .until(driverController.y())
                            .andThen(
                                superstructure
                                    .runGoal(() -> SuperstructureState.NET_SCORE_EJECT)
                                    .until(() -> !superstructure.hasAlgae())
                            )
                            .onlyIf(superstructure::hasAlgae)
                    ),
                superstructure::hasCoral)
        );
    driverController
        .leftTrigger()
        .whileTrue(
            createScoringCommand(false, SuperstructureState.L3)
        );
    driverController
        .back()
        .whileTrue(
            createScoringCommand(false, SuperstructureState.L2)
        );
    driverController
        .rightBumper()
        .whileTrue(
            createScoringCommand(true, SuperstructureState.L4)
        );
    driverController
        .rightTrigger()
        .whileTrue(
            createScoringCommand(true, SuperstructureState.L3)
        );
    driverController
        .leftStick()
        .whileTrue(
            createScoringCommand(true, SuperstructureState.L2)
        );

  }


  private void configureStreamDeckBindings() {
//    streamDeckController
//        .button(1)
//        .onTrue(
//            Commands.runOnce(
//                () -> {
//                  questNavSubsystem.resetPose(
//                      swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp()),
//                      true
//                  );
//                }
//            )
//        );
  }

  public void configureTesterBindings() {
    testerController.a().whileTrue(superstructure.runGoal(() -> SuperstructureState.CORAL_GROUND_INTAKE));
    testerController.y().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.L4)
            .until(testerController.rightTrigger())
            .andThen(
                superstructure.runGoal(() -> SuperstructureState.L4_EJECT)
                    .until(() -> !superstructure.hasCoral())
            )
    );
    testerController.x().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.L3)
            .until(testerController.rightTrigger())
            .andThen(
                superstructure.runGoal(() -> SuperstructureState.L3_EJECT)
                    .until(() -> !superstructure.hasCoral())
            )
    );
    testerController.b().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.L2)
            .until(testerController.rightTrigger())
            .andThen(
                superstructure.runGoal(() -> SuperstructureState.L2_EJECT)
                    .until(() -> !superstructure.hasCoral())
            )
    );
    testerController.povUp().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.P2)
            .until(() -> superstructure.hasAlgae())
    );
    testerController.povDown().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.P1)
            .until(() -> superstructure.hasAlgae())
    );
    testerController.leftBumper().whileTrue(
        superstructure.runGoal(() -> SuperstructureState.NET_SCORE)
            .until(testerController.rightTrigger())
            .andThen(
                superstructure.runGoal(() -> SuperstructureState.NET_SCORE_EJECT)
                    .until(() -> !superstructure.hasAlgae())
            )
    );
    testerController.start().whileTrue(
        Commands.runOnce(() -> {
              destinationSupplier.setCurrentGamePiece(DestinationSupplier.GamePiece.CORAL_SCORING);
            })
            .andThen(
                new ReefAimCommand(swerve, indicatorSubsystem)
            )
    );

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

  /**
   * Helper method to create a scoring command sequence for a given branch and state
   *
   * @param isRightBranch true for right branch, false for left branch
   * @param state         the superstructure state to target
   * @return the command sequence for scoring
   */
  private Command createScoringCommand(boolean isRightBranch, SuperstructureState state) {
    return Commands.sequence(
        Commands.runOnce(() -> destinationSupplier.updateBranch(isRightBranch)),
        Commands.runOnce(() -> destinationSupplier.setStateSetPoint(state)),
        new SuperCycleCommand(swerve, superstructure, indicatorSubsystem, driverController, () -> false)
        // After SuperCycleCommand completes, continue to algae prestate until out of danger zone
        //createDangerZoneExitCommand()
    ).onlyIf(() -> superstructure.hasCoral());
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

    System.out.println("Intake State Decision - HasAlgae: " + hasAlgae + ", InDangerZone: " + inDangerZone);

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
      return superstructure.hasAlgae() && superstructure.indexedCoral();
    } else if (!inDangerZone) {
      // When not in danger zone, we just need coral
      return superstructure.hasCoral();
    } else {
      // When in danger zone (without algae), we need indexed coral
      return superstructure.indexedCoral();
    }
  }

  /**
   * Creates a command to move the superstructure to algae prestate until robot exits danger zone
   * This is used after SuperCycleCommand completes to ensure safe exit from reef area
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
            .until(() -> !isInReefDangerZone())
    ).onlyIf(this::isInReefDangerZone); // Only run if actually in danger zone
  }


  public void robotPeriodic() {
    limelightSubsystem.estimatedPose.ifPresent(estimate -> {
      if (estimate[0] != null) {
        swerve.addVisionMeasurement(
            new Pose3d(estimate[0].pose()), estimate[0].timestampSeconds(), VecBuilder.fill(0.1, 0.1, 0.3, 9999.0)
        );
      }

      if (estimate[1] != null) {
        swerve.addVisionMeasurement(
            new Pose3d(estimate[1].pose()), estimate[1].timestampSeconds(), VecBuilder.fill(0.1, 0.1, 0.3, 9999.0)
        );
      }
    });

    var now = Seconds.of(Timer.getTimestamp());
    RobotStateRecorder.getInstance().putTransform(
        swerve.getEstimatedPose(), now,
        TransformRecorder.kFrameWorld,
        TransformRecorder.kFrameRobot
    );
    RobotStateRecorder.putVelocityRobot(now, swerve.getChassisSpeeds());

    Logger.recordOutput("TF/TWR", RobotStateRecorder.getPoseWorldRobotCurrent());
    Logger.recordOutput("TF/TDR", RobotStateRecorder.getPoseDriverRobotCurrent());
    Logger.recordOutput("TF/VR", RobotStateRecorder.getPoseDriverRobotCurrent());
    Logger.recordOutput("TF/VWR", RobotStateRecorder.getVelocityWorldRobotCurrent());
  }
}
