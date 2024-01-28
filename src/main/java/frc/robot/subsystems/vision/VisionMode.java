package frc.robot.subsystems.vision;

public enum VisionMode {
  AprilTags("limelight-shooter"),
  Notes("limelight-intake");

  public final String name;

  private VisionMode(String llName) {
    this.name = llName;
  }
}
