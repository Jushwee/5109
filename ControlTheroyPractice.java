/*----------------------------------------------------------------------------*/
/* Position PID
 * Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team5109.robot;

/******************************************************************************
 * The following are the Talon libraries used for control of the Talon SRX
 * They are installed by the Phoenix toolsuit located
 * http://www.ctr-electronics.com/hro.html#product_tabs_technical_resources
 */
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
// Check it out- TimedRobot instead of IterativeRobot- this one runs every 20ms no matter what!
public class Robot extends TimedRobot {
	private static final String kDefaultAuto = "Default";
	private static final String kCustomAuto = "My Auto";
	private String m_autoSelected;
	private SendableChooser<String> m_chooser = new SendableChooser<>();
	int _loops=0; // this is a temporary variable used to count off dashboard updates later
	
	//Let's define our constants here.  Why?  Because they're easy to find when tuning.
	//They are implemented below
	private static final double Kp=0.0;
	private static final double Ki=0.0; // .01 * P
	private static final double Kf=0.0;  //no feed-forward on position control
	private static final double Kd=0;
	private static final int IZone=0; 
			

	// I only have one joystick.  here it is:
	Joystick joystick = new Joystick(0);
	
	//I also only have one Talon connected:
	TalonSRX _motor = new TalonSRX(1);
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		m_chooser.addDefault("Default Auto", kDefaultAuto);
		m_chooser.addObject("My Auto", kCustomAuto);
		SmartDashboard.putData("Auto choices", m_chooser);
 
		//Configuration stuff.
		
		_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);	

		/*************************************************************************
		 * Ramps.  The argument is how many seconds it takes to go from 0 to 100%
		 * during voltage control.  There is a ramp for both open loop and closed 
		 * (controlled) loop
		 */
		_motor.configOpenloopRamp(0.2);
		_motor.configClosedloopRamp(0.2);
		
		// Talon setup.  Setting up + and - output for max and nominal.
		//These should never really change
		_motor.configNominalOutputForward(0, 0);
		_motor.configNominalOutputReverse(0, 0);
		_motor.configPeakOutputForward(1,0);
		_motor.configPeakOutputReverse(-1, 0);
		
		//set this if your encoder is hooked up backwards (A<--->B)
		//mine is backwards, so I've set it to true
		_motor.setSensorPhase(true);  
		
		//implementation of tuning constants.  First argument is PID slot.  We're only
		//using slot 0.  If we had multiple tuning constants for the same controller
		// for different purposes, we can also have up to 3 slots pre-programmed
		_motor.config_kF(0, Kf, 0); //No feed-forward on position control
		_motor.config_kP(0, Kp, 0); //P factor!
		_motor.config_kI(0, Ki, 0);
		_motor.config_kD(0, Kd, 0); //note- these are defined above so they're easy to find and change.
		
		//one last term- configIntegralZone
		_motor.config_IntegralZone(0, IZone, 0);
		
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * <p>You can add additional auto modes by adding additional comparisons to
	 * the switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		m_autoSelected = m_chooser.getSelected();
		// m_autoSelected = SmartDashboard.getString("Auto Selector",
		// 		kDefaultAuto);
		System.out.println("Auto selected: " + m_autoSelected);
	}

	/**
	 * This function is called periodically during autonomous.
	 */
	@Override
	public void autonomousPeriodic() {
		switch (m_autoSelected) {
			case kCustomAuto:
				// Put custom auto code here
				break;
			case kDefaultAuto:
			default:
				// Put default auto code here
				break;
		}
	}

	/**
	 * This function is called periodically during operator control.
	 */
	@Override
	public void teleopPeriodic() {
		/**************************************************************************
		 * Here's what I'm doing during teleop:  If you're NOT holding the trigger,
		 * I operate like a normal motor- just piping Y axis of my joystick over
		 * to the motor.  Full stick forward= full motor power.
		 * I'm also reading the Z axis on the joystick.  This gives me a number from
		 * -1 to 1.  When you hit the trigger, I set the target of the Talon
		 * to Z*4096*10 which sets a position target of anywhere from -40 to 40 full
		 * Turns of the encoder wheel.  I have the PID set to get me there as fast
		 * as possible, and the talon takes care of it.  You'll also notice there
		 * are some SmartDashboard numbers thrown up there to check accuracy.
		 */
		double targetPosition=joystick.getZ()*4096*10;
		if (joystick.getTrigger())
		{
			_motor.set(ControlMode.Position, targetPosition);
			if (++_loops >= 10) {
				_loops = 0;
			SmartDashboard.putNumber("Trigger hit", targetPosition);
			SmartDashboard.putNumber("Z value:",joystick.getZ());
			}

			
		}
		//reguluar teleop.  Just give the motor power wherever you have the joystick set
		else if (!joystick.getRawButton(1)) {
			double axis=joystick.getY();
			_motor.set(ControlMode.PercentOutput, axis);
			if (++_loops >= 10) {
				_loops = 0;
			SmartDashboard.putNumber("Trigger hit", targetPosition);
			}
		}
		SmartDashboard.putNumber("Current Pos:", _motor.getSelectedSensorPosition(0));
	}

	/**
	 * This function is called periodically during test mode.
	 */
	@Override
	public void testPeriodic() {
	}
}


