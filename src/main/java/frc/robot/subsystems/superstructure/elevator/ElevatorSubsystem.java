package frc.robot.subsystems.superstructure.elevator;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import lombok.Getter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static frc.robot.RobotConstants.ElevatorConstants;
import static frc.robot.RobotContainer.elevatorIsDanger;

public class ElevatorSubsystem extends SubsystemBase {
    @Getter
    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
    private final LinearFilter currentFilter = LinearFilter.movingAverage(ElevatorConstants.ELEVATOR_ZEROING_FILTER_SIZE);
    @AutoLogOutput(key = "Elevator/currentFilterValue")
    public double currentFilterValue = 0.0;
    @AutoLogOutput(key = "Elevator/zeroing")
    public boolean zeroing = false;
    @Getter@AutoLogOutput(key = "Elevator/setPoint")
    private double wantedPosition = ElevatorConstants.HOLD_EXTENSION_METERS.get();
    @Getter@AutoLogOutput(key = "Elevator/atGoal")
    private boolean atGoal = false;
    private boolean stopProfile = false;

    public ElevatorSubsystem(ElevatorIO io) {
        this.io = io;
    }

    //TODO:Add check Limit，add
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
        //TODO: Delete
        elevatorIsDanger = elevatorIsDanger();
        final boolean runningGoal = 
            !stopProfile &&
            !zeroing;
        if (runningGoal) {
            atGoal = elevatorAtGoal(ElevatorConstants.ELEVATOR_GOAL_TOLERANCE.get());
            io.setElevatorTarget(wantedPosition);
        }else{
            wantedPosition = 0;
        }
    }


    public double getElevatorPosition() {
        return inputs.positionMeters;
    }   

    public void setElevatorPosition(double position) {
        wantedPosition = position;
    }

    public boolean elevatorAtGoal(double offset) {
        return Math.abs(inputs.positionMeters - wantedPosition) < offset;
    }

    //TODO: Change to a command
    public Command zeroElevator() {
        return Commands.startRun(
            () -> {
                stopProfile = true;
                zeroing = true;
            },
            () -> {
                if (RobotBase.isReal()) {
                    if (currentFilterValue <= ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()) {
                        io.setElevatorVoltage(-1);
                    }
                    if (currentFilterValue > ElevatorConstants.ELEVATOR_ZEROING_CURRENT.get()) {
                        io.setElevatorVoltage(0);
                        io.resetElevatorPosition();
                        zeroing = false;
                    }
                } else {
                    io.setElevatorTarget(0);
                    zeroing = false;
                }
            })
            .until(() -> !zeroing)
            .finallyDo(() -> {
                stopProfile = false;
            });
    }

    public boolean elevatorIsDanger() {
        return (inputs.positionMeters < ElevatorConstants.ELEVATOR_MIN_SAFE_HEIGHT.get() - 0.01);
    }
}