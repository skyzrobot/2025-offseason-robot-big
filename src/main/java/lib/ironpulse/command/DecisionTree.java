package lib.ironpulse.command;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import lombok.Getter;
import org.jgrapht.Graph;
import org.jgrapht.graph.DefaultDirectedGraph;
import java.util.function.BooleanSupplier;

public class DecisionTree {
  // fields
  private final Graph<Command, BooleanSupplier> graph;
  private Command rootCommand;
  @Getter
  private Command currentCommand;

  public DecisionTree(Graph<Command, BooleanSupplier> graph) {
    this.graph = graph;
  }

  public DecisionTree(Command rootCommand) {
    this.graph = new DefaultDirectedGraph<>(BooleanSupplier.class);
    this.rootCommand = rootCommand;
  }

  public DecisionTree() {
    this.graph = new DefaultDirectedGraph<>(BooleanSupplier.class);
  }

  public DecisionTree setRoot(Command root) {
    this.rootCommand = root;
    graph.addVertex(root);
    return this;
  }

  public DecisionTree addDecision(
      Command from, Command to,
      BooleanSupplier condition) {
    graph.addVertex(from);
    graph.addVertex(to);
    graph.addEdge(from, to, condition);
    return this;
  }

  public DecisionTree addAlwaysTrueDecision(Command from, Command to) {
    graph.addVertex(from);
    graph.addVertex(to);
    graph.addEdge(from, to, () -> true);
    return this;
  }

  public void reset() {
    currentCommand = null;
  }

  /**
   * Call this every scheduler cycle. It will:
   * 1) schedule root on first tick
   * 2) when currentCommand finishes, test outgoing edges in iteration order
   *    and schedule the first target whose supplier returns true.
   */
  public void tick() {
    CommandScheduler scheduler = CommandScheduler.getInstance();

    // 1. start the root if nothing is running yet
    if (currentCommand == null) {
      if (rootCommand != null) {
        rootCommand.schedule();
        currentCommand = rootCommand;
      }
      return;
    }

    // 2. if currentCommand is done, try transitions
    if (!scheduler.isScheduled(currentCommand)) {
      for (BooleanSupplier edge : graph.outgoingEdgesOf(currentCommand)) {
        if (edge.getAsBoolean()) {
          Command next = graph.getEdgeTarget(edge);
          next.schedule();
          currentCommand = next;
          return;
        }
      }
      // no condition was met, idle
    }
  }

  public boolean isFinished() {
    if (currentCommand == null) {
      return true;
    }
    boolean done = currentCommand.isScheduled();
    boolean terminal = graph.outgoingEdgesOf(currentCommand).isEmpty();
    return done && terminal;
  }

  public Command toCommand() {
    return new DecisionTreeCommand(this);
  }
}
