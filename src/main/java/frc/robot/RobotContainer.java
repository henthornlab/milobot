// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.flatbuffers.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmCommands;
import frc.robot.subsystems.arm.io.RealArmIO;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.drive.Telemetry;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.elevator.io.RealElevatorIO;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands;
import frc.robot.subsystems.multisubsystemcommands.MultiSubsystemCommands.OverallPosition;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import static frc.robot.subsystems.vision.VisionConstants.*;

public class RobotContainer {
  private final Drive drive;
    // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  private final Elevator elevatorSubsystem = new Elevator(new RealElevatorIO());
  private final Arm armSubsystem = new Arm(new RealArmIO());
  private final ElevatorCommands elevatorCommands = new ElevatorCommands(elevatorSubsystem);
  private final ArmCommands armCommands = new ArmCommands(armSubsystem);
  private final Vision vision;

  private final MultiSubsystemCommands multiSubsystemCommands = new MultiSubsystemCommands(elevatorSubsystem, armSubsystem, elevatorCommands, armCommands);

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                    // angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  /* 
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  */
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final CommandXboxController joystick = new CommandXboxController(0);
 
  private final GenericHID buttonbox1 = new GenericHID(1);
  private final JoystickButton button1 = new JoystickButton(buttonbox1, 1);
  private final JoystickButton button2 = new JoystickButton(buttonbox1, 2);
  private final JoystickButton button3 = new JoystickButton(buttonbox1, 3);
  private final JoystickButton button4 = new JoystickButton(buttonbox1, 4);
  private final JoystickButton button5 = new JoystickButton(buttonbox1, 5);
  private final JoystickButton button6 = new JoystickButton(buttonbox1, 6);
  private final JoystickButton button7 = new JoystickButton(buttonbox1, 7);
  private final JoystickButton button8 = new JoystickButton(buttonbox1, 8);
  private final JoystickButton button9 = new JoystickButton(buttonbox1, 9);

  //public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public RobotContainer() {

    switch (HardwareConstants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        
        vision =
                new Vision(
                  drive::addVisionMeasurement,
                  new VisionIOPhotonVision(camera0Name, robotToCamera0),
                  new VisionIOPhotonVision(camera1Name, robotToCamera1));   

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }
    

    
    // All AutoAligns for reef will align to Left position
    
    // NamedCommands.getCommand("L1"); // AutoAlignToReef + ScoreL1
    // NamedCommands.getCommand("L2"); // AutoAlignToReef + ScoreL2
    // NamedCommands.getCommand("L3"); // AutoAlignToReef + ScoreL3
    // NamedCommands.getCommand("L4"); // AutoAlignToReef + ScoreL4

    // NamedCommands.getCommand("Intake"); // AutoAlignToIntake + IntakeUntilHasPiece

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
            
      // Lock to 0° when A button is held
      controller
          .a()
          .whileTrue(
              DriveCommands.joystickDriveAtAngle(
                  drive,
                  () -> -controller.getLeftY(),
                  () -> -controller.getLeftX(),
                  () -> new Rotation2d()));
  
      // Switch to X pattern when X button is pressed
      controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
  
      // Reset gyro to 0° when B button is pressed
      controller
          .b()
          .onTrue(
              Commands.runOnce(
                      () ->
                          drive.setPose(
                              new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                      drive)
                  .ignoringDisable(true)); 


    /*         
    drive.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drive.applyRequest(() -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )); 

    
    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));


    */

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drive.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drive.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drive.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drive.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    //joystick.leftBumper().onTrue(drive.runOnce(() -> drive.seedFieldCentric()));

    //drive.registerTelemetry(logger::telemeterize);

    button1.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.L1));
    button2.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.L2));
    button3.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.Stow));
    button4.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.L3));
    button5.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.L4));
    button6.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.Loading));
    button8.onTrue(multiSubsystemCommands.setOverallSetpoint(OverallPosition.L4_Score));

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
