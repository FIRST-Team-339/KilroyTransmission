// ====================================================================
// FILE NAME: Hardware.java (Team 339 - Kilroy)
//
// CREATED ON: Jan 2, 2011
// CREATED BY: Bob Brown
// MODIFIED ON:
// MODIFIED BY:
// ABSTRACT:
// This file contains all of the global definitions for the
// hardware objects in the system
//
// NOTE: Please do not release this code without permission from
// Team 339.
// ====================================================================
package org.usfirst.frc.team339.Hardware;

import com.ctre.CANTalon;
import org.usfirst.frc.team339.HardwareInterfaces.DoubleThrowSwitch;
import org.usfirst.frc.team339.HardwareInterfaces.IRSensor;
import org.usfirst.frc.team339.HardwareInterfaces.KilroyCamera;
import org.usfirst.frc.team339.HardwareInterfaces.KilroyGyro;
import org.usfirst.frc.team339.HardwareInterfaces.KilroyServo;
import org.usfirst.frc.team339.HardwareInterfaces.MomentarySwitch;
import org.usfirst.frc.team339.HardwareInterfaces.RobotPotentiometer;
import org.usfirst.frc.team339.HardwareInterfaces.SingleThrowSwitch;
import org.usfirst.frc.team339.HardwareInterfaces.UltraSonic;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionFourWheel_old;
import org.usfirst.frc.team339.HardwareInterfaces.transmission.TransmissionMecanum_old;
import org.usfirst.frc.team339.Utils.BallIntake;
import org.usfirst.frc.team339.Utils.Drive;
import org.usfirst.frc.team339.Utils.Shooter;
import org.usfirst.frc.team339.Vision.ImageProcessor;
import org.usfirst.frc.team339.Vision.VisionScript;
import org.usfirst.frc.team339.Vision.operators.ConvexHullOperator;
import org.usfirst.frc.team339.Vision.operators.HSLColorThresholdOperator;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

// -------------------------------------------------------
/**
 * puts all of the hardware declarations into one place. In addition, it makes
 * them available to both autonomous and teleop.
 *
 * @class HardwareDeclarations
 * @author Bob Brown
 * @written Jan 2, 2011
 *          -------------------------------------------------------
 */
public class Hardware
{
// ------------------------------------
// Public Constants
// ------------------------------------


public static double joystickDeadzone = .2;

public static double KILROY_XVII_JOYSTICK_DEADZONE = .2;

public static double KILROY_XVII_JOYSTICK_DIRECTIONAL_DEADZONE = 10.0;

/**
 * denote whether we are running in the lab or not. This will allow us to test
 * in the lab once the robot is bagged
 */
public static boolean runningInLab = false;

public static boolean isRunningOnKilroyXVIII = true; // 18
// -------------------------------------
// Private Constants
// -------------------------------------

// ---------------------------------------
// Hardware Tunables
// ---------------------------------------

public static final double CAMERA_MOUNT_ANGLE = Math.toRadians(65);

// **********************************************************
// DIGITAL I/O CLASSES
// **********************************************************
// ====================================
// PWM classes
// ====================================
public static KilroyServo cameraServo = new KilroyServo(7, 190);
// TODO find actual values

// ------------------------------------
// Jaguar classes
// ------------------------------------

// ------------------------------------
// Talon classes
// ------------------------------------

// changed these to kilroy equipment list 2017
public static TalonSRX rightRearMotor = new TalonSRX(2);

public static TalonSRX rightFrontMotor = new TalonSRX(1);

public static TalonSRX leftRearMotor = new TalonSRX(3);

public static TalonSRX leftFrontMotor = new TalonSRX(4);



// ------------------------------------
// Victor classes
// ------------------------------------

// ====================================
// CAN classes
// ====================================
public static PowerDistributionPanel pdp = new PowerDistributionPanel(
        0);


// ====================================
// Relay classes
// ====================================


// ====================================
// Digital Inputs
// ====================================
// ------------------------------------
// Single and double throw switches
// ------------------------------------

// ------------------------------------
// Gear Tooth Sensors
// ------------------------------------

// ------------------------------------
// Encoders
// ------------------------------------

public static Encoder leftRearEncoder = new Encoder(10, 11);

public static Encoder rightRearEncoder = new Encoder(12, 13);

public static Encoder leftFrontEncoder = new Encoder(14, 15);

public static Encoder rightFrontEncoder = new Encoder(16, 17);



// -----------------------
// Wiring diagram
// -----------------------
// Orange - Red PWM 1
// Yellow - White PWM 1 Signal
// Brown - Black PWM 1 (or PWM 2)
// Blue - White PWM 2 Signal
// For the AMT103 Encoders UNVERIFIED
// B - White PWM 2
// 5V - Red PWM 1 or 2
// A - White PWM 1
// X - index channel, unused
// G - Black PWM 1 or 2
// see http://www.cui.com/product/resource/amt10-v.pdf page 4 for Resolution
// (DIP Switch) Settings (currently all are off)

// -------------------------------------
// Red Light/IR Sensor class
// -------------------------------------
// ====================================
// I2C Classes
// ====================================

// **********************************************************
// SOLENOID I/O CLASSES
// **********************************************************
// ====================================
// Compressor class - runs the compressor
// ====================================
// public static Compressor compressor = new Compressor();

// ====================================
// Pneumatic Control Module
// ====================================

// ====================================
// Solenoids
// ====================================
// ------------------------------------
// Double Solenoids
// ------------------------------------

// ------------------------------------
// Single Solenoids
// ------------------------------------

// **********************************************************
// ANALOG I/O CLASSES
// **********************************************************
// ====================================
// Analog classes
// ====================================
// ------------------------------------
// Gyro class
// ------------------------------------

// -------------------------------------
// Potentiometers
// -------------------------------------
// -------------------------------------
// TODO swap with us
public static RobotPotentiometer delayPot = new RobotPotentiometer(1,
        270);// TODO max //
// degree value

// -------------------------------------
// Sonar/Ultrasonic
// -------------------------------------


// **********************************************************
// roboRIO CONNECTIONS CLASSES
// **********************************************************
// -------------------------------------
// Axis/USB Camera class
// -------------------------------------

// Note: If causing problems, replace "USB_Camera_0" w/ "cam0", and
// "USB_Camera_1" w/ "cam1"

public static KilroyCamera axisCamera = new KilroyCamera(true);// TODO change


public static VisionScript visionScript = new VisionScript(
        new HSLColorThresholdOperator(34, 117, 34, 181, 9, 160), /*
                                                                  * 79, 210, 7,
                                                                  * 214, 33,
                                                                  * 255)
                                                                  */// (76, 200,
                                                                   // 71,
        // new RemoveSmallObjectsOperator(1, true), // // 255, 50,255),
        new ConvexHullOperator(false));




public static ImageProcessor imageProcessor = new ImageProcessor(
        axisCamera, visionScript);
// -------------------------------------
// declare the USB camera server and the
// USB camera it serves
// -------------------------------------

// **********************************************************
// DRIVER STATION CLASSES
// **********************************************************

// ------------------------------------
// DriverStations class
// ------------------------------------
/**
 * The software object representing the driver station.
 */
public static final DriverStation driverStation = DriverStation
        .getInstance();

// ------------------------------------
// Joystick classes
// ------------------------------------

/**
 * The left joystick controlling the drive train.
 */
public static Joystick leftDriver = new Joystick(0);

/**
 * The right joystick controlling the drive train.
 */
public static Joystick rightDriver = new Joystick(1);

/**
 * The left joystick controlling misc operations on the robot.
 */
public static Joystick leftOperator = new Joystick(2);

/**
 * The right joystick controlling misc operations on the robot.
 */
public static Joystick rightOperator = new Joystick(3);


// **********************************************************
// Kilroy's Ancillary classes
// **********************************************************

// -------------------------------------
// PID tuneables
// -------------------------------------

// -------------------------------------
// PID classes
// -------------------------------------

// ------------------------------------
// Transmission class
// ------------------------------------
// public static Transmission_old transmission = new Transmission_old(
// rightRearMotor, leftRearMotor, rightRearEncoder,
// leftRearEncoder);

// ------------------------------------
// Drive system
// ------------------------------------

public static TransmissionMecanum_old mecanumDrive = new TransmissionMecanum_old(
        rightFrontMotor, rightRearMotor, leftFrontMotor, leftRearMotor);

public static TransmissionFourWheel_old tankDrive = new TransmissionFourWheel_old(
        rightFrontMotor, rightRearMotor, leftFrontMotor, leftRearMotor);


// =====================================================================
// Drive classes
// =====================================================================


/**
 * are we using mecanum? set false for tank drive
 */
public static boolean isUsingMecanum = true;

/**
 * are we using 2 joysticks?
 */
public static boolean twoJoystickControl = false;


// -------------------
// Assembly classes (e.g. forklift)
// -------------------

// ------------------------------------
// Utility classes
// ------------------------------------
/**
 * Default timer.
 */
public static final Timer kilroyTimer = new Timer();

public static final Timer autoStateTimer = new Timer();


/**
 * Default motor safety
 * 
 * @TODO We REALLY need to fix the motor safety...
 */
// public static final MotorSafetyHelper leftRearMotorSafety = new
// MotorSafetyHelper(
// leftRearMotor);


/**
 * Default motor safety
 */
// public static final MotorSafetyHelper rightRearMotorSafety = new
// MotorSafetyHelper(
// rightRearMotor);


// public static final MotorSafetyHelper rightFrontMotorSafety = new
// MotorSafetyHelper(
// rightFrontMotor);

// public static final MotorSafetyHelper leftFrontMotorSafety = new
// MotorSafetyHelper(
// leftFrontMotor);

public static final int MINIMUM_AXIS_CAMERA_BRIGHTNESS = 6;

} // end class
