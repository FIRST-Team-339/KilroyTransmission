/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
// ====================================================================
// FILE NAME: Autonomous.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 13, 2015
// CREATED BY: Nathanial Lydick
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file is where almost all code for Kilroy will be
// written. Some of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// Init() - Initialization code for autonomous mode
// should go here. Will be called each time the robot enters
// autonomous mode.
// -----------------------------------------------------
// Periodic() - Periodic code for autonomous mode should
// go here. Will be called periodically at a regular rate while
// the robot is in autonomous mode.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.robot;

import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.Utils.Drive;
import org.usfirst.frc.team339.Utils.Drive.AlignReturnType;
import org.usfirst.frc.team339.Utils.Shooter.turnReturn;
import org.usfirst.frc.team339.Utils.Shooter.turnToGoalReturn;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Relay.Value;


/**
 * An Autonomous class.
 * This class <b>beautifully</b> uses state machines in order to periodically
 * execute instructions during the Autonomous period.
 * 
 * This class contains all of the user code for the Autonomous part
 * of the
 * match, namely, the Init and Periodic code
 * 
 * 
 * @author Michael Andrzej Klaczynski
 * @written at the eleventh stroke of midnight, the 28th of January,
 *          Year of our LORD 2016. Rewritten ever thereafter.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public class Autonomous
{
// ==========================================
// AUTO STATES
// ==========================================
/**
 * The overarching states of autonomous mode.
 * Each state represents a set of instructions the robot must execute
 * periodically at a given time.
 * These states are mainly used in the runMainStateMachine() method
 * 
 */
private static enum MainState
    {
    /**
     * Reset Encoder Values, determine alliance, determine delay time,
     * determine path
     */
    INIT,
    /**
     * Delays anywhere between 0 and 5 seconds, controlled by a potentiometer on
     * the robot
     */
    DELAY_BEFORE_START,
    /**
     * Accelerates so we don't jerk our encoders.
     */
    ACCELERATE,
    /**
     * Drives up the the gear.
     */
    DRIVE_FORWARD_TO_CENTER,
    /**
     * Drive to the side of the airship at normal speed. See
     * DRIVE_FORWARD_TO_CENTER.
     */
    DRIVE_FORWARD_TO_SIDES,
    /**
     * Stops the robot after we've driven to the side of the airship, before we
     * turn to face the gear peg
     */
    BRAKE_BEFORE_TURN_TO_GEAR_PEG,
    /**
     * Turn towards the peg deposit place on the airship
     */
    TURN_TO_GEAR_PEG,
    /**
     * Stops our angular motion after we turn the the gear so we're at least
     * partially aligned to the gear
     */
    BRAKE_AFTER_TURN_TO_GEAR_PEG,
    /**
     * Align with the vision strips to deposit gear
     */
    DRIVE_TO_GEAR_WITH_CAMERA,

    /**
     * Drive up to the gear peg when we don't see any blobs.
     */
    DRIVE_CAREFULLY_TO_PEG,
    /**
     * Stops us moving up to the peg so we don't spear ourselves on it.
     */
    BRAKE_UP_TO_PEG,
    /**
     * 
     */
    TURN_TURRET_OUT_OF_THE_WAY,
    /**
     * Currently unused, may be used to regain vision targets, or make sure the
     * gear is on the spring. It's a bit of a wildcard at the moment.
     */
    WIGGLE_WIGGLE,
    /**
     * We've got the gear on the peg (probably), but we're waiting for the human
     * player to pull the gear out of our thingy.
     */
    WAIT_FOR_GEAR_EXODUS,
    /**
     * Wait after we detect the gear leaving our mechanism to make sure it's
     * clear before we try and drive back.
     */
    DELAY_AFTER_GEAR_EXODUS,
    /**
     * Back away from whatever peg we're currently on.
     */
    DRIVE_AWAY_FROM_PEG,
    /**
     * On paths where we go for the hopper, Turn so we're facing it
     */
    TURN_TO_HOPPER,
    /**
     * On paths where we go for the hopper, drive until we slam into it.
     */
    DRIVE_UP_TO_HOPPER,
    /**
     * Since we drive backwards to get into range, we need to turn around to
     * fire.
     */
    TURN_TO_FACE_GOAL,
    /**
     * On paths where we fire, back away from the gear peg and towards the
     * boiler until we're in range to fire.
     */
    DRIVE_TO_FIRERANGE,
    /**
     * Use the camera to figure out if we're in range of the top boiler
     */
    DRIVE_INTO_RANGE_WITH_CAMERA,
    /**
     * Turn the turret so it's facing the boiler
     */
    ALIGN_TO_FIRE,
    /**
     * Empty out the hopper or fire until time out
     */
    FIRE,
    /**
     * Quit the path.
     */
    DONE
    }

/**
 * The state machine controlling which auto path we are taking
 * 
 * @author Alex Kneipp
 */
private static enum AutoProgram
    {
    /**
     * The state we start in, runs once.
     */
    INIT,

    /**
     * The path where we start in the middle and (try to) place the gear on the
     * center peg.
     */
    CENTER_GEAR_PLACEMENT,

    /**
     * The path where we start on the right side of the field and drive
     * forwards, turn and place the gear, and try and turn around to attempt to
     * fire. (Based on whether or not we are on the red or blue alliance)
     */
    RIGHT_PATH,

    /**
     * The path where we start on the left side of the field and drive
     * forwards, turn and place the gear, and try and turn around to attempt to
     * fire. (Based on whether or not we are on the red or blue alliance)
     */
    LEFT_PATH,

    /**
     * We are done with this auto path! Yay! (or aww depending on if it
     * worked...)
     */
    DONE
    }



// ==================================
// VARIABLES
// ==================================
/**
 * Temporarily holds each state for the return type for Strafing to Gear
 * 
 * Found in DRIVE_TO_GEAR_WITH_CAMERA states.
 */
private static Drive.AlignReturnType cameraState = Drive.AlignReturnType.NO_BLOBS;

// ==========================================
// TUNEABLES
// ==========================================

/**
 * If we are using mecanum, this is the number of DEGREES the robot will offset
 * while using StrafeToGear.
 * 
 * If we are using Tank drive (for some strange reason) this should be changed
 * to a percentage that will be offset for each side of the robot.
 */
private static final double ALIGN_CORRECT_VAR = 30;// 45

/**
 * How fast we will be driving during all of auto, in percent.
 */
private static final double DRIVE_SPEED = .45;

/**
 * Determines what value we set the motors backwards to in order to brake, in
 * percentage.
 */
private static final double BRAKE_SPEED = .3;

/**
 * Determines how long we should set the motors backwards in order to brake
 * using timeBrake in the Drive class, in percentage.
 */
private static final double BRAKE_TIME = .2;

/**
 * The deadband for considering whether or not we are aligned to the target(s).
 * Change the pixel value, and the /getHorizontalResolution will change it to
 * relative coordinates.
 */
private static final double ALIGN_DEADBAND = 7 // +/- pixels
        / Hardware.axisCamera.getHorizontalResolution();

/**
 * Where we say the "center" of the camera is / where we want to correct TO.
 */
private static final double ALIGN_ACCEPTED_CENTER = .5; // Relative coordinates;
                                                        // half the screen = .5

/**
 * How far away we are in inches when we stop moving towards the goal
 * 
 * Used in DRIVE_CARFULLY_TO_GEAR as well as StrafeToGear
 */
private static final int STOP_DISTANCE_TO_GEAR = 14;

/**
 * How fast we should accelerate in the accelerate state, in seconds.
 */
private static final double TIME_TO_ACCELERATE = .4;

/**
 * User-Initialization code for autonomous mode should go here. Will be
 * called once when the robot enters autonomous mode.
 *
 * @author Nathanial Lydick
 *
 * @written Jan 13, 2015
 */

public static void init ()
{
    // reset encoders
    Hardware.leftFrontEncoder.reset();
    Hardware.leftRearEncoder.reset();
    Hardware.rightFrontEncoder.reset();
    Hardware.rightRearEncoder.reset();
    // motors
} // end Init

/**
 * User Periodic code for autonomous mode should go here. Will be called
 * periodically at a regular rate while the robot is in autonomous mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public static void periodic ()
{
    // The main state machine for figuring out which path we are following
   
} // end Periodic

} // end class
