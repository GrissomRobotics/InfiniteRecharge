/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveByUltrasonic;
import frc.robot.commands.ExpelCellByTime;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Belt;
import frc.robot.subsystems.OutputSystem;
import frc.robot.subsystems.Spinner;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html

public class AutonomousDefault extends SequentialCommandGroup {
  /**
   * Creates a new defualt.
   */
  public AutonomousDefault(DriveSubsystem driveTrain, Spinner spinner, Belt belt, OutputSystem outputSystem) {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    //super();
    addCommands(
    
      //new DriveByUltrasonic(driveTrain, spinner, 1, 50),

      new ExpelCellByTime(belt, outputSystem, 2)

    );
  }
}
