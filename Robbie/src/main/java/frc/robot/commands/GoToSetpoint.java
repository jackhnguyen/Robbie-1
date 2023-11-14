package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class GoToSetpoint extends CommandBase{
    SuperStructure superStructure;
    SuperStructureState state;

    public GoToSetpoint(SuperStructure superStructure, SuperStructureState state){
        this.superStructure = superStructure;
        this.state = state;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setState(state);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();
    }
}
