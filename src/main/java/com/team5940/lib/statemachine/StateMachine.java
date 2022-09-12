package com.team5940.lib.statemachine;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// The StateMachine class is to be extended by any classes that would otherwise be subsystems in standard wpilib command-based projects. 
// It provides methods that make it easier to write statemachine-based code while staying somewhat true to the wpilib command-based paradigm.
public abstract class StateMachine extends SubsystemBase {

    // Variable to track whether the StateMachine has been initialized
    private boolean mInitialized = false;

    // The handle() method runs once
    // It's for initializing motors and variables that would otherwise not work in the constructor.
    protected abstract void init();

    // The handle(Request r) method is to be overriden by any subclasses.
    // For any requesed action, this method will change instance variables and system states
    // to ensure that the correct state output is being applied
    protected abstract void handle(Request r);

    // The onLoop() method is to be overriden by any subclasses. 
    // This method applies outputs and transitions based on state variables
    // It is periodically called every loop cycle. 
    protected abstract void onLoop();

    // The log() method is to overriden by any subclasses.
    // This method logs anything that the user so pleases. 
    // It is periodically called every loop cycle. 
    protected abstract void log();

    // Periodically calls init() once and onLoop() and log() every loop cycle. 
    @Override
    public void periodic() {
        if (!mInitialized) {
            init();
            mInitialized = true;
        }
        onLoop();
        log();
    }

}
