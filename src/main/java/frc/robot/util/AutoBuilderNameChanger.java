package frc.robot.util;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.List;

public class AutoBuilderNameChanger {
  public static SendableChooser<Command> buildNameChangedAutoChooser() {
    if (!AutoBuilder.isConfigured()) {
      throw new RuntimeException(
          "AutoBuilder was not configured before attempting to build an auto chooser");
    }

    SendableChooser<Command> chooser = new SendableChooser<>();
    List<String> autoNames = AutoBuilder.getAllAutoNames();

    List<PathPlannerAuto> options = new ArrayList<>();

    for (String autoName : autoNames) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);
      options.add(auto);
    }

    chooser.setDefaultOption("None", Commands.none());

    options.forEach(auto -> chooser.addOption(getUpdatedAutoName(auto.getName()), auto));

    return chooser;
  }

  public static String getUpdatedAutoName(String defaultName) {
    switch (defaultName) {
      case "AmpSide_AmpSideNote_End":
        return "START: AMP \n -> TWO PIECE";
      case "AmpSide_AmpSideNote_Midline1_End":
        return "START: AMP \n -> THREE PIECE";
      case "AmpSide_AmpSideNote_Midline1_Midline2_End":
        return "START: AMP \n -> FOUR PIECE, AMP NOTE AND MIDLINE";
      case "AmpSide_AmpSideNote_CenterNote_SourceSideNote_End":
        return "START: AMP \n -> FULL WING AUTO";
      case "SourceSide_End":
        return "START: SOURCE \n -> GTFOOTW";
      case "SourceSide_SourceSideNote_End":
        return "START: SOURCE \n -> TWO PIECE";
      case "SourceSide_Midline5_Midline4_End":
        return "START: SOURCE \n -> MIDLINE 5, 4";
      case "SourceSide_Midline4_Midline5_End":
        return "START: SOURCE \n -> MIDLINE 4, 5";
      case "OpponentSource_Midline4_Midline3_End":
        return "START: OPPONENT SOURCE \n -> MIDLINE 4, 3";
      case "OpponentSource_Midline5_Midline4_Midline3_End":
        return "START: OPPONENT SOURCE \n -> MIDLINE 5, 4, 3";
      case "SourceSide_SourceSideNote_Midline5_End":
        return "START: SOURCE \n -> URI AUTO";
      case "SourceSide_SourceSideNote_Midline5_Midline4_End":
        return "START: SOURCE \n -> URI + 1";
      case "SourceSide_SourceSideNote_CenterNote_AmpSideNote_End":
        return "START: SOURCE \n -> FULL WING AUTO";
      case "Center_AmpSideNote_CenterNote_SourceSideNote_End":
        return "START: CENTER \n -> START AMP, FULL WING AUTO";
      case "Center_SourceSideNote_CenterNote_AmpSideNote_End":
        return "START: CENTER \n -> START SOURCE, FULL WING AUTO";
      case "Center_CenterNote_Midline3_SourceSideNote_End":
        return "START: CENTER \n ROBOTEERS COMPLEMENT";
      case "Center_CenterNote_Midline3_End":
        return "START: CENTER \n ROBOTEERS COMPLEMENT - 1";
      case "Leave":
        return "START: OPPONENT SOURCE \n -> LEAVE";
    }

    return defaultName;
  }
}
