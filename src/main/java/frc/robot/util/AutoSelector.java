package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutoSelector extends VirtualSubsystem {

  private static final int maxQuestions = 4;
  private static final AutoRoutine defaultRoutine =
      new AutoRoutine("Do Nothing", List.of(), Commands.none());

  private final LoggedDashboardChooser<AutoRoutine> routineChooser;
  private final List<StringPublisher> questionPublishers;
  private final List<SwitchableChooser> questionChoosers;

  private AutoRoutine lastRoutine;
  private List<AutoQuestionResponse> lastResponses;

  public AutoSelector(String key) {
    routineChooser = new LoggedDashboardChooser<>(key + "/Routine");
    routineChooser.addDefaultOption(defaultRoutine.name(), defaultRoutine);
    lastRoutine = defaultRoutine;
    lastResponses = List.of();

    // Publish questions and choosers
    questionPublishers = new ArrayList<>();
    questionChoosers = new ArrayList<>();
    for (int i = 0; i < maxQuestions; i++) {
      var publisher =
          NetworkTableInstance.getDefault()
              .getStringTopic("/SmartDashboard/" + key + "/Question #" + Integer.toString(i + 1))
              .publish();
      publisher.set("N/A");
      questionPublishers.add(publisher);
      questionChoosers.add(
          new SwitchableChooser(key + "/Question #" + Integer.toString(i + 1) + "Chooser"));
    }
  }

  public void addRoutine(String name, List<AutoQuestion> questions, Command command) {
    if (questions.size() > maxQuestions) {
      throw new RuntimeException(
          "Auto routine contained more than "
              + Integer.toString(maxQuestions)
              + " questions: "
              + name);
    }
    routineChooser.addOption(name, new AutoRoutine(name, questions, command));
  }

  public Command getCommand() {
    return lastRoutine.command();
  }

  public List<AutoQuestionResponse> getResponses() {
    return lastResponses;
  }

  @Override
  public void periodic() {
    if (DriverStation.isAutonomousEnabled() && lastRoutine != null && lastResponses == null) {
      return;
    }

    var selectedRoutine = routineChooser.get();
    if (selectedRoutine == null) {
      return;
    }

    if (!selectedRoutine.equals(lastRoutine)) {
      var questions = selectedRoutine.questions();
      for (int i = 0; i < maxQuestions; i++) {
        if (i < questions.size()) {
          questionPublishers.get(i).set(questions.get(i).question());
          questionChoosers
              .get(i)
              .setOptions(
                  questions.get(i).responses().stream()
                      .map((AutoQuestionResponse response) -> response.toString())
                      .toArray(String[]::new));
        } else {
          questionPublishers.get(i).set("");
          questionChoosers.get(i).setOptions(new String[] {});
        }
      }
    }

    lastRoutine = selectedRoutine;
    lastResponses = new ArrayList<>();
    for (int i = 0; i < lastRoutine.questions().size(); i++) {
      String responseString = questionChoosers.get(i).get();
      lastResponses.add(
          responseString == null
              ? lastRoutine.questions().get(i).responses().get(0)
              : AutoQuestionResponse.valueOf(responseString));
    }
  }

  private static final record AutoRoutine(
      String name, List<AutoQuestion> questions, Command command) {}

  public static record AutoQuestion(String question, List<AutoQuestionResponse> responses) {}

  public static enum AutoQuestionResponse {
    YES,
    NO,
    AMP_SIDE,
    CENTER,
    SOURCE_SIDE,
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    SIX,
    SEVEN
  }
}
