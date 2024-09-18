// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.command.DefaultClimb;
import frc.robot.command.DefaultIntake;
import frc.robot.command.DefaultShooter;
import frc.robot.command.DefaultSwerve;
import frc.robot.command.ResetClimb;
import frc.robot.command.autolime.AutoAlignTags;
import frc.robot.command.autolime.AutoDriveAndTrackNote;
import frc.robot.command.autolime.AutoIntake;
import frc.robot.command.autolime.AutoShootSmart;
import frc.robot.command.autolime.NoteRotationAlign;
import frc.robot.command.autolime.autoSequences.CenterAuto;
import frc.robot.command.autolime.autoSequences.LeftAuto;
import frc.robot.command.autolime.autoSequences.RightAuto;
import frc.robot.command.autolime.autoSequences.ThreeNoteCenterAuto;
import frc.robot.subsytems.Arms;
import frc.robot.subsytems.Intake;
import frc.robot.subsytems.Shooter;
import frc.robot.subsytems.SwerveSubsystem;

public class RobotContainer {

    // controllers
    private final Joystick primaryJoy = new Joystick(0);
    private final XboxController secondaryController = new XboxController(1);
    // TODO subsystems
    private final SwerveSubsystem swerveSub = new SwerveSubsystem();
    private final Arms Arms = new Arms();
    private final Shooter shooter = new Shooter();
    private final Intake mouth = new Intake(shooter);
    private final DefaultIntake intakeTransport = new DefaultIntake(mouth, secondaryController, shooter, primaryJoy);
    private final DefaultShooter shootCommand = new DefaultShooter(primaryJoy, secondaryController, shooter, mouth);
    // commands
    private final DefaultSwerve defaultSwerve = new DefaultSwerve(primaryJoy, swerveSub);
    private final DefaultClimb climbCommand = new DefaultClimb(primaryJoy, Arms);
    // private final PowerDistribution pdp = new PowerDistribution(1,
    // ModuleType.kRev);
    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        swerveSub.setDefaultCommand(defaultSwerve);
        shooter.setDefaultCommand(shootCommand);
        mouth.setDefaultCommand(intakeTransport);
        Arms.setDefaultCommand(climbCommand);
        configureBindings();

        // Shuffleboard.getTab("Debug").add(pdp);

        NamedCommands.registerCommand("SmartIntake", new AutoIntake(mouth));
        // NamedCommands.registerCommand("Shoot", new AutoShootSmart(shooter, mouth));
        // NamedCommands.registerCommand("Shoot", Commands.sequence(
        // new AutoAlignTags(swerveSub).withTimeout(1),
        // new StopCommand(swerveSub),
        // new AutoShootSmart(shooter, mouth)
        // ));
        NamedCommands.registerCommand("Shoot", new InstantCommand());
        var makeSmarterShoot = (Supplier<Command>) () -> Commands.sequence(
                // new AutoAlignTags(swerveSub).withTimeout(1),
                // new StopCommand(swerveSub),
                new AutoShootSmart(shooter, mouth));

        var sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> swerveSub.runVolts(voltage),
                        null, // No log consumer, since data is recorded by URCL
                        swerveSub));

        Shuffleboard.getTab("Debug").add("GoHome", AutoBuilder.pathfindToPose(new Pose2d(1.5, 5.55, new Rotation2d()),
                new PathConstraints(.75, .5, Math.PI / 6, Math.PI / 6)).onlyIf(swerveSub::hasSeenMegatag));
        Shuffleboard.getTab("Drive").add("GoHome", AutoBuilder.pathfindToPose(new Pose2d(1.5, 5.55, new Rotation2d()),
                new PathConstraints(.75, .5, Math.PI / 6, Math.PI / 6)).onlyIf(swerveSub::hasSeenMegatag));

        // AutoBuilder.followPath(PathPlannerPath.fromPathPoints(null, null, null))

        autoChooser.addOption("Line", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Line")));
        autoChooser.addOption("Stupid", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Stupid")));
        autoChooser.addOption("Rotate90", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Rotate90")));
        autoChooser.addOption("Triangle", AutoBuilder.followPath(PathPlannerPath.fromPathFile("Triangle")));
        // autoChooser.addOption("PPRightAuto", AutoBuilder.buildAuto("RightAuto"));
        // autoChooser.addOption("PP4Auto", AutoBuilder.buildAuto("4Auto"));
        // autoChooser.addOption("PP4Smooth", AutoBuilder.buildAuto("4Smooth"));

        autoChooser.addOption("PPCenterAndRight", Commands.sequence(
                makeSmarterShoot.get(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("1GotoMiddleNote")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("1ComeFromMiddleNote")),
                makeSmarterShoot.get(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("1GotoRightNote")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("1ComeFromRightNote")),
                makeSmarterShoot.get()));


        autoChooser.addOption("PPCenterAndLeft", Commands.sequence(
                makeSmarterShoot.get(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2GotoMiddleNote")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2ComeFromMiddleNote")),
                makeSmarterShoot.get(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2GotoLeftNote")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2ComeFromLeftNote")),
                makeSmarterShoot.get()));

        autoChooser.addOption("PP4SmoothCommand", Commands.sequence(
                makeSmarterShoot.get(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2GotoMiddleNote")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2ComeFromMiddleNote")),
                makeSmarterShoot.get(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2GotoLeftNote")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2ComeFromLeftNote")),
                makeSmarterShoot.get(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2GotoRightNote")),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2ComeFromRightNote")),
                makeSmarterShoot.get(),
                AutoBuilder.followPath(PathPlannerPath.fromPathFile("2ComeFromLastShotToHome"))));
        // autoChooser.addOption("Quasi Forward",
        // sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Quasi Backward",
        // sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        // autoChooser.addOption("Dynamic Forward",
        // sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        // autoChooser.addOption("Dynamic Backward",
        // sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
        // Shuffleboard.getTab("Tune").add("SysID Drivetrain", autoChooser);

        autoChooser.addOption("right", new RightAuto(swerveSub, shooter, mouth));
        autoChooser.addOption("center", new CenterAuto(swerveSub, shooter, mouth));
        autoChooser.addOption("left", new LeftAuto(swerveSub, shooter, mouth));
        autoChooser.addOption("SmartShoot", makeSmarterShoot.get());
        autoChooser.addOption("3_Note_Center", new ThreeNoteCenterAuto(swerveSub, shooter, mouth));
        // autoChooser.addOption("right",new ThreeAutoToRuleThemAll(swerveSub, shooter, mouth));
        Shuffleboard.getTab("Drive").add(autoChooser);
        Shuffleboard.getTab("Debug").add("ResetClimb", new ResetClimb(Arms));

        // Shooter shooterSub = new Shooter();
        // AutoDrive step = new AutoDrive(swerveSub, 0, 0); // TODO
        // push commands to pathweaver auto
        // NamedCommands.registerCommand("drive", step);

        // autoChooser = AutoBuilder.buildAutoChooser();

        // Shuffleboard.getTab("autoChooser").add(autoChooser);
        swerveSub.zeroYaw();
        swerveSub.resetOmetry(new Pose2d(1, 1, new Rotation2d()));


        // warms up json parsing
        new Thread(() -> {
            LimelightHelpers.getLatestResults("limelight-back");
        }).start();

    }

    private void configureBindings() {

        new JoystickButton(primaryJoy, 3).whileTrue(new AutoAlignTags(swerveSub));
        new JoystickButton(primaryJoy, 10).whileTrue(new NoteRotationAlign(swerveSub));
        // new JoystickButton(primaryJoy, 11).whileTrue(AutoBuilder.pathfindToPose(new
        // Pose2d(1.5, 5.55, new Rotation2d()),
        // new PathConstraints(1, .75, Math.PI / 3, Math.PI /
        // 3.5)).onlyIf(swerveSub::hasSeenMegatag));
        // new JoystickButton(primaryJoy, 8).whileTrue(new ProxyCommand(() -> {
        // System.err.println("Creating new history follower");
        // if (swerveSub.history.size() == 0) {
        // return new InstantCommand();
        // }
        // var path = new ArrayList<PathPoint>();
        // for (var i = swerveSub.history.size() - 1; i > 0; i--) {
        // path.add(swerveSub.history.get(i));
        // }
        // swerveSub.history.clear();
        // return AutoBuilder.followPath(PathPlannerPath.fromPathPoints(
        // path,
        // new PathConstraints(3, 3, Math.PI, Math.PI),
        // new GoalEndState(0, new Rotation2d())));
        // }));
        new JoystickButton(primaryJoy, 9).whileTrue(new AutoDriveAndTrackNote(swerveSub, 2.5, 0.3));
    }

    public Command getAutonomousCommand() {
        // final var path = PathPlannerPath.fromPathFile("Line");
        // return AutoBuilder.followPath(path);

        // return new AutoDrive(swerveSub, Units.feetToMeters(8), .1);

        var command = autoChooser.getSelected();


        // // Command command = null;
        if (command != null) {
            return command;
        } else {
            return new InstantCommand();
        }
        // return new OneAutoToRuleThemAll(swerveSub, shooter, mouth);
    }

    // public void resetFieldOrientation() {
    //   // swerveSub.zeroYaw();
    // }
}