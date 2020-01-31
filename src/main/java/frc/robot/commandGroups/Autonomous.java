/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.commands.DriveByUltrasonic;
import frc.robot.commands.RotateToAngle;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous.
   */

  public Autonomous(DriveSubsystem driveTrain) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    //super();
    addCommands(

      new DriveByUltrasonic(driveTrain, 12, 0.5),

      new RotateToAngle(driveTrain, 90, 0.5),

      new DriveByUltrasonic(driveTrain, 94, 0.5),

      new RotateToAngle(driveTrain, 90, 0.5),

      new DriveByUltrasonic(driveTrain, 1, 0.5)
    );
  }
}