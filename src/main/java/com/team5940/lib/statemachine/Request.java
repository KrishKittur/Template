package com.team5940.lib.statemachine;

import edu.wpi.first.wpilibj2.command.CommandBase;

// The Request class is to be extended by all user-facing classes that alter the system state. 
public abstract class Request extends CommandBase {

    // Stores the statemachine that the Request is "attatched" to.
    StateMachine stateMachine;

    // Constructs a new request object, setting fields and adding requirements.
    public Request(StateMachine stateMachine) {
        this.stateMachine = stateMachine;
        addRequirements(stateMachine);
    }

    // Calls the handle method for the given request
    @Override
    public void initialize() {
        stateMachine.handle(this);
    }

    // This command only needs to run once, so the isFinished() method automatically returns true
    @Override
    public boolean isFinished() {
        return true;
    }

}
