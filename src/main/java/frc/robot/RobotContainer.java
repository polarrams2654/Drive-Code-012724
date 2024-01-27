// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem drivetrain = new DriveTrainSubsystem();
  public final Joystick driveJoystick;
  public final Joystick turnJoystick;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
      driveJoystick = new Joystick(0);
      turnJoystick = new Joystick(1);
      
      drivetrain.setDefaultCommand(
      new DriveCommand(
          // Forward velocity supplier.
          driveJoystick::getY,
          // Sideways velocity supplier.
          driveJoystick::getX,
          // Rotation velocity supplier.
          turnJoystick::getX,
          drivetrain
      )
  );
    // Configure the trigger bindings
    configureBindings();
  }
  private void configureBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}
