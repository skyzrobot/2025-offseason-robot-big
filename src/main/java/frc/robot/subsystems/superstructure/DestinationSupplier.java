package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.commands.aimSequences.AimGoalSupplier;
import frc.robot.subsystems.swerve.Swerve;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.subsystems.superstructure.SuperstructureState.*;

public class DestinationSupplier {
    private static DestinationSupplier instance;
    @Getter
    @Setter
    public boolean useSuperCycle = true;
    Swerve swerve;
    @Getter
    private controlMode currentControlMode = controlMode.AUTO;
    @Getter
    @Setter
    private int targetTagID = 0;
    private boolean coralRight = false;
    @Getter
    private AlgaeScoringMode algaeScoringMode = AlgaeScoringMode.NET;
    @Getter
    @AutoLogOutput(key = "DestinationSupplier/currentStateSetpointCoral")
    private SuperstructureState currentStateSetpointCoral = L2;
    @Getter
    @AutoLogOutput(key = "DestinationSupplier/currentStateSetpointAlgae")
    private SuperstructureState currentStateSetpointAlgae = P1;
    @Getter
    @AutoLogOutput(key = "DestinationSupplier/CurrentPiece")
    private GamePiece currentGamePiece = GamePiece.CORAL_SCORING;

    private DestinationSupplier() {
        swerve = Swerve.getInstance();
    }

    public static DestinationSupplier getInstance() {
        if (instance == null) {
            instance = new DestinationSupplier();
        }
        return instance;
    }

    /**
     * Determines if it's safe to raise the elevator based on robot position
     *
     * @param robotPose Current pose of the robot
     * @param rightReef Whether targeting the right reef relative to the AprilTag
     * @return true if the robot is within safe distance to raise elevator, false otherwise
     */
    public static boolean isSafeToRaise(Pose2d robotPose, boolean rightReef) {
        Pose2d tag = AimGoalSupplier.getNearestTag(robotPose);
        Pose2d goal = AimGoalSupplier.getFinalCoralTarget(tag, rightReef);
        return goal.getTranslation().getDistance(robotPose.getTranslation()) < RobotConstants.ReefAimConstants.RAISE_LIMIT_METERS.get();
    }

    public void setStateSetPoint(SuperstructureState stateSetPoint) {
        switch (stateSetPoint) {
            case L1_INTAKE_SIDE, L2, L3, L4:
                currentStateSetpointCoral = stateSetPoint;
                break;
            case P1, P2:
                currentStateSetpointAlgae = stateSetPoint;
                break;
            default:
                throw new IllegalStateException("Unexpected value: " + currentStateSetpointCoral);
        }
    }

    public SuperstructureState getPreState() {
        return currentGamePiece == GamePiece.ALGAE_INTAKING ? 
            currentStateSetpointAlgae : 
            currentStateSetpointCoral;
    }

    public SuperstructureState getShootState() {
        return currentGamePiece == GamePiece.ALGAE_INTAKING ? 
            getShootAlgaeState() : 
            getShootCoralState();
    }

    private SuperstructureState getShootCoralState() {
        return switch (currentStateSetpointCoral) {
            case L1_SHOOT_SIDE -> L1_SHOOT_SIDE_EJECT;
            case L1_INTAKE_SIDE -> L1_INTAKE_SIDE_EJECT;
            case L2 -> L2_EJECT;
            case L3 -> L3_EJECT;
            case L4 -> L4_EJECT;
            case NET_SCORE -> NET_SCORE_EJECT;
            default -> throw new IllegalStateException("Unexpected coral state: " + currentStateSetpointCoral);
        };
    }

    private SuperstructureState getShootAlgaeState() {
        return switch (algaeScoringMode) {
            case NET -> NET_SCORE_EJECT;
            case PROCESSOR -> throw new IllegalStateException("Processor mode not implemented for algae scoring");
        };
    }

    public void updateBranch(boolean coralRight) {
        this.coralRight = coralRight;
        Logger.recordOutput("DestinationSupplier/Pipe", coralRight);
        SmartDashboard.putString("DestinationSupplier/Pipe", coralRight ? "Right" : "Left");
    }

    public boolean getCurrentBranch() {
        return coralRight;
    }

    public void updatePokeSetpointByTag(int tagNumber) {
        switch (tagNumber) {
            case 6, 8, 10, 17, 19, 21:
                setStateSetPoint(SuperstructureState.P1);
                break;
            case 7, 9, 11, 18, 20, 22:
                setStateSetPoint(SuperstructureState.P2);
                break;
            default:
                System.out.println("Tag number does not correspond to a valid elevator setpoint.");
        }
    }

    public Command setCurrentGamePiece(DestinationSupplier.GamePiece currentGamePiece) {
        return Commands.runOnce(
                () ->{
                    this.currentGamePiece = currentGamePiece;
                    if (currentGamePiece == GamePiece.ALGAE_INTAKING) {
                        updatePokeSetpointByTag(AimGoalSupplier.getNearestTagID(swerve.getLocalizer().getCoarseFieldPose(Timer.getFPGATimestamp())));
                    }
                }
        );
    }

    public void switchUseSuperCycle() {
        useSuperCycle = !useSuperCycle;
        Logger.recordOutput("DestinationSupplier/UseSuperCycle", useSuperCycle);
        SmartDashboard.putBoolean("DestinationSupplier/UseSuperCycle", useSuperCycle);
    }

    public enum controlMode {
        MANUAL, SEMI, AUTO
    }

    public enum GamePiece {
        CORAL_SCORING,
        ALGAE_INTAKING
    }

    public enum AlgaeScoringMode {
        NET,
        PROCESSOR
    }
}
