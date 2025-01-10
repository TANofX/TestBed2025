package frc.robot.commands.auto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import java.util.Optional;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class AutoBuildingBlocks {
  
  Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();

  static Command allianceConditionalCommand(Command redCommand, Command blueCommand) {
    return new ConditionalCommand(redCommand, blueCommand, () -> (DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : null) == Alliance.Red);
  }
} 