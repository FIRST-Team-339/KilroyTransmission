/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved. */
/* Open Source Software - may be modified and shared by FRC teams. The code */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project. */
/*----------------------------------------------------------------------------*/
// ====================================================================
// FILE NAME: Kilroy.java (Team 339 - Kilroy)
//
// CREATED ON: Oct 19, 2012
// CREATED BY: Bob Brown
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file is where almost all code for Kilroy will be
// written. All of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// autonomousInit() - Initialization code for autonomous mode
// should go here. Will be called each time the robot enters
// autonomous mode.
// disabledInit() - Initialization code for disabled mode should
// go here. This function will be called one time when the
// robot first enters disabled mode.
// robotInit() - Robot-wide initialization code should go here.
// It will be called exactly 1 time.
// teleopInit() - Initialization code for teleop mode should go here.
// Will be called each time the robot enters teleop mode.
// -----------------------------------------------------
// autonomousPeriodic() - Periodic code for autonomous mode should
// go here. Will be called periodically at a regular rate while
// the robot is in autonomous mode.
// disabledPeriodic() - Periodic code for disabled mode should go here.
// Will be called periodically at a regular rate while the robot
// is in disabled mode.
// teleopPeriodic() - Periodic code for teleop mode should go here.
// Will be called periodically at a regular rate while the robot
// is in teleop mode.
// -----------------------------------------------------
// autonomousContinuous() - Continuous code for autonomous mode should
// go here. Will be called repeatedly as frequently as possible
// while the robot is in autonomous mode.
// disabledContinuous() - Continuous code for disabled mode should go
// here. Will be called repeatedly as frequently as possible while
// the robot is in disabled mode.
// teleopContinuous() - Continuous code for teleop mode should go here.
// Will be called repeatedly as frequently as possible while the
// robot is in teleop mode.
// -----------------------------------------------------
// Other functions not normally used
// startCompetition() - This function is a replacement for the WPI
// supplied 'main loop'. This should not normally be written or
// used.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.robot;

import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
import org.usfirst.frc.team339.Hardware.Hardware;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.vision.AxisCamera.Resolution;
import edu.wpi.first.wpilibj.vision.AxisCamera.WhiteBalance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
// -------------------------------------------------------
/**
 * declares all the code necessary to extend the IterativeRobot class. These are
 * all the methods needed to run Kilroy during a match
 *
 * @author Bob Brown
 * @written Jan 2, 2011 -------------------------------------------------------
 */
public class Robot extends IterativeRobot
{
// =================================================
// private data for the class
// =================================================
// -------------------------------------------------------

/**
 * Initialization code for autonomous mode should go here. Will be called
 * once when the robot enters autonomous mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void autonomousInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started AutonousInit().");
    // =========================================================
    // User code goes below here
    // =========================================================
    // -------------------------------------
    // Call the Autonomous class's Init function,
    // which contains the user code.
    // -------------------------------------
    Autonomous.init();
    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed AutonousInit().");
} // end autonomousInit

// -------------------------------------------------------
/**
 * Non-User Periodic code for autonomous mode should go here. Will be called
 * periodically at a regular rate while the robot is in autonomous mode.
 * This in turn calls the Autonomous class's Periodic function, which is
 * where the user code should be placed.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void autonomousPeriodic ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    // System.out.println("Started AutonomousPeriodic().");
    // =========================================================
    // User code goes below here
    // =========================================================
    // -------------------------------------
    // Call the Autonomous class's Periodic function,
    // which contains the user code.
    // -------------------------------------\
    Autonomous.periodic();
    // =========================================================
    // User code goes above here
    // =========================================================
    // feed all motor safeties
}// end autonomousPeriodic
 // -------------------------------------------------------

/**
 * Initialization code for disabled mode should go here. Will be called once
 * when the robot enters disabled mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void disabledInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started DisabledInit().");
    // =========================================================
    // User code goes below here
    // =========================================================
    // motors
    // Hardware.rightFrontMotor.setInverted(true);
    // Hardware.rightRearMotor.setInverted(true);
    // Hardware.leftFrontMotor.setInverted(true);
    // Hardware.leftRearMotor.setInverted(true);
    // mecanum
    Hardware.mecanumDrive.setMecanumJoystickReversed(false);
    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed DisabledInit().");
} // end disabledInit

// -------------------------------------------------------
/**
 * Periodic code for disabled mode should go here. Will be called
 * periodically at a regular rate while the robot is in disabled mode. Code
 * that can be "triggered" by a joystick button can go here. This can set up
 * configuration things at the driver's station for instance before a match.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void disabledPeriodic ()
{
    // -------------------------------------
    // Watch dog code used to go here.
    // -------------------------------------
    // =========================================================
    // User code goes below here
    // =========================================================
    // motors
    Hardware.leftFrontMotor.set(0);
    Hardware.rightFrontMotor.set(0);
    Hardware.leftRearMotor.set(0);
    Hardware.rightRearMotor.set(0);
    // motor safty
    // Hardware.leftFrontMotorSafety.feed();
    // Hardware.leftRearMotorSafety.feed();
    // Hardware.rightFrontMotorSafety.feed();
    // Hardware.rightFrontMotorSafety.feed();
    // =========================================================
    // User code goes above here
    // =========================================================
} // end disabledPeriodic
  // -------------------------------------------------------

/**
 * This function is run when the robot is first started up and should be
 * used for any initialization code for the robot.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void robotInit ()
{
    // -------------------------------------
    // Watch dog code used to go here.
    // -------------------------------------
    // =========================================================
    // User code goes below here
    // =========================================================
    // reset encoders
    Hardware.leftFrontEncoder.reset();
    Hardware.leftRearEncoder.reset();
    Hardware.rightFrontEncoder.reset();
    Hardware.rightRearEncoder.reset();
    // change stuff based on kilroy version
   
    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println(
            "Kilroy XVIII is started.  All hardware items created.");

} // end
  // robotInit

// -------------------------------------------------------
/**
 * Non-User initialization code for teleop mode should go here. Will be
 * called once when the robot enters teleop mode, and will call the Teleop
 * class's Init function, where the User code should be placed.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void teleopInit ()
{
    // ---------------------------------------
    // start setup - tell the user we are beginning
    // setup
    // ---------------------------------------
    System.out.println("Started teleopInit().");
    // =========================================================
    // User code goes below here
    // =========================================================
    Teleop.init();
    // =========================================================
    // User code goes above here
    // =========================================================
    // ---------------------------------------
    // done setup - tell the user we are complete
    // setup
    // ---------------------------------------
    System.out.println("Completed TeleopInit().");
} // end teleopInit

// -------------------------------------------------------
/**
 * Non-User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode, and
 * will in turn call the Teleop class's Periodic function.
 *
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
@Override
public void teleopPeriodic ()
{
    // -------------------------------------
    // Call the Teleop class's Periodic function,
    // which contains the user code.
    // -------------------------------------
    Teleop.periodic();
} // end teleopPeriodic
  // -------------------------------------------------------

/**
 * Initialization code for test mode should go here. Will be called once
 * when the robot enters test mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2015
 *          -------------------------------------------------------
 */
@Override
public void testInit ()
{
    // =========================================================
    // User code goes below here
    // =========================================================

    // =========================================================
    // User code goes above here
    // =========================================================

} // end testInit
  // -------------------------------------------------------

/**
 * Periodic code for test mode should go here. Will be called periodically
 * at a regular rate while the robot is in test mode.
 *
 * @author Bob Brown
 * @written Jan 2, 2015
 *          -------------------------------------------------------
 */
@Override
public void testPeriodic ()
{
    // =========================================================
    // User code goes below here
    // =========================================================

    // =========================================================
    // User code goes above here
    // =========================================================

} // end testPeriodic

// ==========================================
// TUNEABLES
// ==========================================

/**
 * The percentage we want the motors to run at while we are in first gear
 */
public static final double KILROY_XVIII_FIRST_GEAR_PERCENTAGE = .8;

/**
 * The percentage we want the motors to run at while we are in second gear
 */
public static final double KILROY_XVIII_SECOND_GEAR_PERCENTAGE = 1;

/**
 * The percentage we want the motors to run at while we are in first gear
 * for Kilroy XVII/ Ballbot
 */
public static final double KILROY_XVII_FIRST_GEAR_PERCENTAGE = 1.0;

/**
 * The percentage we want the motors to run at while we are in second gear
 * for Kilroy XVII/ Ballbot
 */
public static final double KILROY_XVII_SECOND_GEAR_PERCENTAGE = 1.0;

/**
 * Distance per pulse for kilroy XVIII
 */

public static final double ENCODER_DISTANCE_PER_PULSE_KILROY_XVIII = 0.069;

/**
 * Distance per pulse on kilroy XVII
 * 
 */

public static final double ENCODER_DISTANCE_PER_PULSE_KILROY_XVII = .0197;

// pid stuff
public static double shooterP = .3;

public static double shooterI = .0000084;

public static double shooterD = 2.0;

} // end class
