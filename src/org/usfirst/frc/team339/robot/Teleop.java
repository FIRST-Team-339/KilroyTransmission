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
// written. All of these functions are functions that should
// override methods in the base class (IterativeRobot). The
// functions are as follows:
// -----------------------------------------------------
// Init() - Initialization code for teleop mode
// should go here. Will be called each time the robot enters
// teleop mode.
// -----------------------------------------------------
// Periodic() - Periodic code for teleop mode should
// go here. Will be called periodically at a regular rate while
// the robot is in teleop mode.
// -----------------------------------------------------
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.robot;

import com.ctre.CANTalon.FeedbackDevice;
import org.usfirst.frc.team339.Hardware.Hardware;
import org.usfirst.frc.team339.Utils.Drive;
import org.usfirst.frc.team339.Utils.Shooter;
import org.usfirst.frc.team339.Utils.Shooter.turnToGoalReturn;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class contains all of the user code for the Autonomous part of the
 * match, namely, the Init and Periodic code
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */
public class Teleop
{
/**
 * User Initialization code for teleop mode should go here. Will be called
 * once when the robot enters teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */

public static void init ()
{
    // --------------------------------------
    // initialize all encoders here
    // --------------------------------------
    Hardware.leftFrontEncoder.reset();
    Hardware.rightFrontEncoder.reset();
    Hardware.rightRearEncoder.reset();
    Hardware.leftRearEncoder.reset();
    // --------------------------------------
    // initialize all motors here
    // --------------------------------------
    Hardware.leftRearMotor.set(0.0);
    Hardware.rightRearMotor.set(0.0);
    Hardware.rightFrontMotor.set(0.0);
    Hardware.leftFrontMotor.set(0.0);
    // ---------------------------------------
    // Servo init
    // ---------------------------------------
   
} // end Init

static double tempSetpoint = 0.0;

/**
 * User Periodic code for teleop mode should go here. Will be called
 * periodically at a regular rate while the robot is in teleop mode.
 *
 * @author Nathanial Lydick
 * @written Jan 13, 2015
 */

public static void periodic ()
{
    // print values from hardware items
    printStatements();

} // end
  // Periodic


private static boolean isDrivingStraight = false;

private static boolean isBraking = false;

private static Drive.AlignReturnType alignValue = Drive.AlignReturnType.MISALIGNED;

private static Shooter.turnReturn turnValue = Shooter.turnReturn.SUCCESS;

private static boolean isTurningToGoal = false;

private static turnToGoalReturn turnToGoalValue = Shooter.turnToGoalReturn.SUCCESS;

private static double rotationValue = 0.0;

private static boolean isTurningGimbal = false;

private static boolean isAligning = false;

private static boolean previousFireButton = false;

private static boolean isStrafingToTarget = false;

private static boolean preparingToFire = false;

private static double movementSpeed = 0.3;

private static boolean firing = false;

private static boolean hasPressedFour = false;

/**
 * stores print statements for future use in the print "bank", statements
 * are commented out when not in use, when you write a new print statement,
 * "deposit" the statement in the correct "bank" do not "withdraw"
 * statements, unless directed to.
 * 
 * NOTE: Keep the groupings below, which coorespond in number and order as
 * the hardware declarations in the HARDWARE class
 * 
 * @author Ashley Espeland
 * @written 1/28/16
 * 
 *          Edited by Ryan McGee Also Edited by Josef Liebl
 * 
 */
public static void printStatements ()
{
    // =================================
    // Motor controllers
    // prints value of the motors
    // =================================
    // System.out.println("Delay Pot: " + Hardware.delayPot.get(0, 5));
    // System.out.println("Right Front Motor Controller: "
    // + Hardware.rightFrontMotor.get());
    // System.out.println("Left Front Motor Controller: " +
    // Hardware.leftFrontMotor.get());
    // System.out.println("Right Rear Motor Controller: " +
    // Hardware.rightRearMotor.get());
    // System.out.println("Left Rear Motor Controller: " +
    // Hardware.leftRearMotor.get());
    // System.out.println("Flywheel thingy thing: "
    // + Hardware.shooter.calculateRPMToMakeGoal(9.25) * .5);
    // System.out.println("Flywheel thingy thing speed really: "
    // + Hardware.shooterMotor.get());
    // System.out.println(Hardware.backupOrFireOrHopper.isOn());
    // System.out
    // .println("Flywheel Motor: " + Hardware.shooterMotor.get());
    //
    // System.out.println("Intake Motor: " + Hardware.intakeMotor.get());
    // if (Hardware.rightOperator.getRawButton(11)) {
    // Hardware.elevatorMotor.setSpeed(1);
    // System.out.println("Elevator Motor: " +
    // Hardware.elevatorMotor.get());
    // }
    // System.out.println("Turret Spark: " + Hardware.gimbalMotor.get());

    // =================================
    // CAN items
    // prints value of the CAN controllers
    // =================================
    // Hardware.CAN.printAllPDPChannels();

    // =================================
    // Relay
    // prints value of the relay states
    // =================================
    // if (Hardware.ringlightSwitch.isOnCheckNow())
    // {
    // System.out.println("Ring light relay is On");
    // }
    // else if (!Hardware.ringlightSwitch.isOnCheckNow())
    // {
    // System.out.println("Ring light relay is Off");
    // }
    // =================================
    // Digital Inputs
    // =================================
    // ---------------------------------
    // Switches
    // prints state of switches
    // ---------------------------------
    // System.out.println("Gear Limit Switch: "
    // + Hardware.gearLimitSwitch.isOn());
    // System.out.println("Backup or fire: " +
    // Hardware.backupOrFireOrHopper.isOn());
    // System.out.println("Enable Auto: " +
    // Hardware.enableAutonomous.isOn());
    // System.out.println(
    // "Path Selector: " + Hardware.pathSelector.getPosition());

    // System.out.println("Right UltraSonic distance from bumper: "
    // + Hardware.rightUS.getDistanceFromNearestBumper());
    // System.out.println("Right UltraSonic refined distance: "
    // + Hardware.rightUS.getRefinedDistanceValue());
    // System.out.println("Right UltraSonic raw distance: "
    // + Hardware.rightUS.getValue());

    // ---------------------------------
    // Encoders
    // prints the distance from the encoders
    // ---------------------------------
    // Hardware.rightFrontEncoder.get());
    // System.out.println("Right Front Encoder Distance: " +
    // Hardware.autoDrive.getRightRearEncoderDistance());
    // System.out.println("Right Rear Encoder: " +
    // Hardware.rightRearEncoder.get());
    // System.out.println("Right Rear Encoder Distance: " +
    // Hardware.autoDrive.getRightRearEncoderDistance());
    // System.out.println("Left Front Encoder: " +
    // Hardware.leftFrontEncoder.get());
    // System.out.println("Left Front Encoder Distance: " +
    // Hardware.autoDrive.getLeftFrontEncoderDistance());
    // System.out.println("Left Rear Encoder: " +
    // Hardware.leftRearEncoder.get());
    // System.out.println("Left Rear Encoder Distance: " +
    // Hardware.autoDrive.getLeftFrontEncoderDistance());
    // ---------------------------------
    // Red Light/IR Sensors
    // prints the state of the sensor
    // ---------------------------------
    // System.out.println("Ball IR: " + Hardware.ballLoaderSensor.isOn());
    // =================================
    // Pneumatics
    // =================================
    // ---------------------------------
    // Compressor
    // prints information on the compressor
    // ---------------------------------
    // There isn't one at the moment

    // ---------------------------------
    // Solenoids
    // prints the state of solenoids
    // ---------------------------------
    // There are none at the moment
    // That seems familiar...

    // =================================
    // Analogs
    // =================================

    // GYRO
    // System.out.println("Gyro: " + Hardware.driveGyro.getAngle());

    // System.out.println("Ultrasonic = "
    // + Hardware.rightUS.getDistanceFromNearestBumper());

    // System.out.println("Delay Pot: " + Hardware.delayPot.get());
    // ---------------------------------
    // pots
    // where the pot is turned to
    // ---------------------------------
    // System.out.println("Delay Pot Degrees" + Hardware.delayPot.get());
    // =================================
    // Connection Items
    // =================================
    // ---------------------------------
    // Cameras
    // prints any camera information required
    // ---------------------------------
    // System.out.println("Expected center: " + CAMERA_ALIGN_CENTER);
    //
    // Hardware.imageProcessor.processImage();
    // Hardware.imageProcessor.filterBlobsInYRange(1, .9);
    // if (Hardware.imageProcessor.getLargestBlob() != null)
    // {
    // System.out.println("Center of Mass: " + Hardware.imageProcessor
    // .getLargestBlob().center_mass_y);
    // }
    // else
    // {
    // System.out.println("NO BLOBS!");
    // }
    // if (Hardware.imageProcessor.getNthSizeBlob(1) != null)
    // System.out
    // .println("Actual center: " + ((Hardware.imageProcessor
    // .getNthSizeBlob(0).center_mass_x
    // + Hardware.imageProcessor
    // .getNthSizeBlob(1).center_mass_x)
    // / 2.0)
    // / Hardware.axisCamera
    // .getHorizontalResolution());
    //
    // System.out.println("Deadband: " + CAMERA_ALIGN_DEADBAND);
    // =================================
    // Driver station
    // =================================
    // ---------------------------------
    // Joysticks
    // information about the joysticks
    // ---------------------------------
    // System.out.println("Right Joystick: " +
    // Hardware.rightDriver.getDirectionDegrees());
    // System.out.println("Left Operator: " +
    // Hardware.leftOperator.getDirectionDegrees());
    // System.out.println("Right Operator: " +
    // Hardware.rightOperator.getDirectionDegrees());
    // System.out.println("Twist: " + Hardware.leftDriver.getTwist());
    // =================================
    // Driver station
    // =================================
    // ---------------------------------
    // Joysticks
    // information about the joysticks
    // ---------------------------------
    // System.out.println("Left Joystick Direction: " +
    // Hardware.leftDriver.getDirectionDegrees());
    // if (Hardware.leftDriver.getTrigger())
    // {
    // System.out.println("Twist: " + Hardware.leftDriver.getTwist());
    // }
    // System.out.println("Left Joystick: " + Hardware.leftDriver.getY());
    // System.out.println("Right Joystick: " + Hardware.rightDriver.getY());
    // System.out.println("Left Operator: " + Hardware.leftOperator.getY());
    // System.out.println("Right Operator: " +
    // Hardware.rightOperator.getY());
    // =================================
    // Kilroy ancillary items
    // =================================
    // ---------------------------------
    // timers
    // what time does the timer have now
    // ---------------------------------
} // end printStatements

/*
 * =============================================== 
 * Constants
 * ===============================================
 */

} // end class
