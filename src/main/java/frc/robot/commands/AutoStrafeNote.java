package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class AutoStrafeNote extends PIDCommand {

  private DriveSubsystem ds;

  static NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision").getSubTable("Camera_Module_v1");
  static NetworkTableEntry x = table.getEntry("targetPixelsX");
private static double getHori() {
  return (x.getDouble(160)/160)-1;
}

  public AutoStrafeNote(DriveSubsystem ds) {
    super(
        new PIDController(0.1, 0, 0),
        // This should return the measurement
        () -> -getHori(),
        // This should return the setpoint (can also be a constant)
        () -> 0,
        // This uses the output
        output -> {
          var o = MathUtil.clamp(output, -.7, .7);
          ds.drivePercent(0, o, 0, false, 0, 0);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    addRequirements(ds);
    this.ds = ds;
  }


  @Override
  public void execute() {
      // TODO Auto-generated method stub
      super.execute();
  System.out.println(""+getHori());
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
