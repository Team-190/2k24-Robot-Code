package frc.robot;

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
      case "AmpSide_AmpSideNote_CenterNote_SourceSideNote_End":
        return "2 Spike Amp";
      case "AmpSide_AmpSideNote_End":
        return "1 Spike Amp";
      case "AmpSide_MidelineSweep_End":
        return "MidlineSweep Amp";
      case "AmpSide_Midline1Note_Midline2Note_AmpSideNote":
        return "2 Midline 1 Spike Amp";
      case "AmpSide_Midline1Note_Midline2Note_AmpSideNote_CenterNote":
        return "2 Midline 2 Spike Amp";
      case "AmpSide_Midline1Note_Midline2Note_AmpSideNote_CenterNote_SourceSideNote":
        return "2 Midline 3 Spike Amp";
      case "AmpSide_Midline1Note_Midline2Note_Midline3Note_AmpSideNote":
        return "3 Midline 1 Spike Amp";
      case "AmpSide_Midline1Note_Midline2Note_Midline3Note_AmpSideNote_CenterNote":
        return "3 Midline 2 Spike Amp";
      case "AmpSide_Midline1Note_Midline2Note_Midline3Note_AmpSideNote_CenterNote_SourceSideNote":
        return "3 Midline 3 Spike Amp";
      case "Center_AmpSideNote_End":
        return "AmpSideSpike Center";
      case "Center_CenterNote_End":
        return "CenterSpike Center";
      case "Center_CenterNote_Midline3Note_End":
        return "1 Midline 1 Spike Center";
      case "Center_SourceSideNote_CenterNote_AmpSideNote_End":
        return "1 Midline 1 Spike Center";
      case "Center_SourceSideNote_End":
        return "SourceSpike Center";
      case "SourceSide_MidlineSweep_End":
        return "MidlineSweep Source";
      case "SourceSide_SourceSideNote_CenterSideNote_AmpSideNote_End":
        return "3 Spike Source";
      case "SourceSide_SourceSideNote_End":
        return "1 Spike Source";
      case "SourceSide_SourceSideNote_Mideline5_End":
        return "1 Midline 1 Spike Source";
      case "SourceSide_SourceSideNote_Mideline5_Mideline4_End":
        return "2 Midline 1 Spike Source";
      case "SourceSide_SourceSideNote_Mideline5_Mideline4_Midline3_End":
        return "3 Midline 1 Spike Source";
    }

    return defaultName;
  }
}
