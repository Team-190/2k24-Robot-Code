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
        return "START: AMP \n -> FULL WING AUTO";
      case "AmpSide_AmpSideNote_End":
        return "START: AMP \n -> AMP SIDE NOTE";
      case "AmpSide_MidelineSweep_End":
        return "START: AMP \n -> MIDLINE SWEEP";
      case "AmpSide_Midline1Note_Midline2Note_AmpSideNote":
        return "START: AMP \n -> MIDLINE 1 \n -> MIDLINE 2 \n -> AMP SIDE NOTE";
      case "AmpSide_Midline1Note_Midline2Note_AmpSideNote_CenterNote":
        return "START: AMP \n -> MIDLINE 1 \n -> MIDLINE 2 \n -> AMP SIDE NOTE \n -> CENTER NOTE";
      case "AmpSide_Midline1Note_Midline2Note_AmpSideNote_CenterNote_SourceSideNote":
        return "START: AMP \n -> MIDLINE 1 \n -> MIDLINE 2 \n -> AMP SIDE NOTE \n -> CENTER NOTE \n -> SOURCE SIDE NOTE";
      case "AmpSide_Midline1Note_Midline2Note_Midline3Note_AmpSideNote":
        return "START: AMP \n -> MIDLINE 1 \n -> MIDLINE 2 \n -> MIDLINE 3 \n -> AMP SIDE NOTE";
      case "AmpSide_Midline1Note_Midline2Note_Midline3Note_AmpSideNote_CenterNote":
        return "START: AMP \n -> MIDLINE 1 \n -> MIDLINE 2 \n -> MIDLINE 3 \n -> AMP SIDE NOTE \n -> CENTER NOTE";
      case "AmpSide_Midline1Note_Midline2Note_Midline3Note_AmpSideNote_CenterNote_SourceSideNote":
        return "START: AMP \n -> MIDLINE 1 \n -> MIDLINE 2 \n -> MIDLINE 3 \n -> AMP SIDE NOTE \n -> CENTER NOTE \n -> SOURCE SIDE NOTE";
      case "Center_AmpSideNote_End":
        return "START: CENTER \n -> AMP SIDE NOTE";
      case "Center_CenterNote_End":
        return "START: CENTER \n -> CENTER NOTE";
      case "Center_CenterNote_Midline3Note_End":
        return "START: CENTER \n -> CENTER NOTE \n -> MIDLINE 3";
      case "Center_SourceSideNote_CenterNote_AmpSideNote_End":
        return "START: CENTER \n -> FULL WING AUTO";
      case "Center_SourceSideNote_End":
        return "START: CENTER \n -> SOURCE SIDE SPIKE";
      case "SourceSide_MidlineSweep_End":
        return "START: SOURCE \n -> MIDLINE SWEEP";
      case "SourceSide_SourceSideNote_CenterSideNote_AmpSideNote_End":
        return "START: SOURCE \n -> FULL WING AUTO";
      case "SourceSide_SourceSideNote_End":
        return "START: SOURCE \n -> SOURCE SIDE SPIKE";
      case "SourceSide_SourceSideNote_Mideline5_End":
        return "START: SOURCE \n -> SOURCE SIDE SPIKE \n -> MIDLINE 5";
      case "SourceSide_SourceSideNote_Mideline5_Mideline4_End":
        return "START: SOURCE \n -> SOURCE SIDE SPIKE \n -> MIDLINE 5 \n -> MIDLINE 4";
      case "SourceSide_SourceSideNote_Mideline5_Mideline4_Midline3_End":
        return "START: SOURCE \n -> SOURCE SIDE SPIKE \n -> MIDLINE 5 \n -> MIDLINE 4 \n -> MIDLINE 3";
    }

    return defaultName;
  }
}
