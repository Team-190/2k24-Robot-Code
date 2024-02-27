package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.AutoSelector.AutoQuestionResponse;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

public class AutoCommands {
  private final Supplier<List<AutoQuestionResponse>> responses;

  public AutoCommands(Supplier<List<AutoQuestionResponse>> responses) {
    this.responses = responses;
  }

  public Command onePiece() {
    return Commands.select(
        Map.of(
            AutoQuestionResponse.AMP_SIDE,
            AutoBuilder.buildAuto("AmpSide_MidelineSweep_End"),
            AutoQuestionResponse.SOURCE_SIDE,
            AutoBuilder.buildAuto("SourceSide_MidlineSweep_End")),
        () -> responses.get().get(0));
  }

  public Command twoPiece() {
    return Commands.select(
        Map.of(
            AutoQuestionResponse.AMP_SIDE,
            AutoBuilder.buildAuto("AmpSide_AmpSideNote_End"),
            AutoQuestionResponse.CENTER,
            Commands.select(
                Map.of(
                    AutoQuestionResponse.AMP_SIDE,
                    AutoBuilder.buildAuto("Center_AmpSideNote_End"),
                    AutoQuestionResponse.CENTER,
                    AutoBuilder.buildAuto("Center_CenterNote_End"),
                    AutoQuestionResponse.SOURCE_SIDE,
                    AutoBuilder.buildAuto("Center_SourceSideNote_End")),
                () -> responses.get().get(1)),
            AutoQuestionResponse.SOURCE_SIDE,
            AutoBuilder.buildAuto("SourceSide_SourceSideNote_End")),
        () -> responses.get().get(0));
  }

  public Command threePiece() {
    return Commands.select(
        Map.of(
            AutoQuestionResponse.CENTER,
            AutoBuilder.buildAuto("Center_CenterNote_Midline3Note_End"),
            AutoQuestionResponse.SOURCE_SIDE,
            AutoBuilder.buildAuto("SourceSide_SourceSideNote_Mideline5_End")),
        () -> responses.get().get(0));
  }
}
