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
        return "START: AMP \n -> AMP SIDE NOTE";
      case "AmpSide_AmpSideNote_Midline1_End":
        return "START: AMP \n -> AMP SIDE NOTE \n -> MIDLINE 1";
      case "AmpSide_AmpSideNote_Midline1_Midline2_End":
        return "START: AMP \n -> AMP SIDE NOTE \n -> MIDLINE 1 \n -> MIDLINE 2";
      case "SourceSide_End":
        return "START: SOURCE \n -> GTFOOTW";
      case "SourceSide_Midline5_Midline4_End":
        return "START: SOURCE \n -> MIDLINE 5 \n -> MIDLINE 4";
      case "SourceSide_SourceSideNote_End":
        return "START: SOURCE \n -> SOURCE SIDE NOTE";
      case "SourceSide_SourceSideNote_Midline5_End":
        return "START: SOURCE \n -> SOURCE SIDE NOTE \n -> MIDLINE 5";
      case "Leave":
        return "START: OPPONENT SOURCE \n -> LEAVE";
      case "SourceSide_Midline4_Midline3_End":
        return "START: SOURCE \n -> MIDLINE 4 \n -> MIDLINE 3";
    }

    return defaultName;
  }
}
