package frc.robot.autons;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class CollectCubeTMBlue extends SequentialCommandGroup {
  
  /*
   * Beijing 2008:
   * - Mascot is pandas thing
   * - Comp included Osaka and Toronto
   * - Meant to avoid political reasoning
   * - Increase in tourism and more criticism
   * - Successful
   * 
   * Pyeongchang 2018:
   * - Munich was comp
   * - Did not really help with economy
   * - Increased korean culture awareness
   * 
   * Brazil 2016:
   * - Nice weather
   * - Infrastructure was a double-sided coin
   * - Increased tourism and GDP
   */

  private List<PathPlannerTrajectory> pathGroup;
  private HashMap<String, Command> eventMap = new HashMap<>();

  public CollectCubeTMBlue(DriveSubsystem driveS) {

    // Gets the path from the pathplanner generated file
    pathGroup = PathPlanner.loadPathGroup(
      "CollectCubeTMBlue",
      new PathConstraints(AutoConstants.MAX_SPEED_METERS_PER_SECOND, AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
    );

    // Adds commands to each marker
    eventMap.put("intakeOn", new PrintCommand("Turn on intake"));
    eventMap.put("intakeOff", new PrintCommand("Turn off intake"));
    eventMap.put("outtakeBlock", new PrintCommand("Outtake the block"));

    var autoBuilder = driveS.genRamseteAutoBuilder(eventMap);

    addCommands(
      Commands.runOnce(() -> driveS.resetOdometry(pathGroup.get(0).getInitialPose())),
      autoBuilder.fullAuto(pathGroup),
      Commands.runOnce(() -> driveS.driveVolts(0.0, 0.0))
    );

  }

}
