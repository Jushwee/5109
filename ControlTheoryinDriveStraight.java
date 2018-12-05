package org.usfirst.frc.team5109.robot;

//Latest code as of 4/11/18
import java.math.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import java.util.logging.Level;
import java.util.logging.Logger;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.*;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix.motorcontrol.*;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
    TalonSRX leftMotor1 =  new TalonSRX(6); //6, 10, 5, 4
    TalonSRX leftMotor2 =  new TalonSRX(10);
    TalonSRX rightMotor1 =  new TalonSRX(5);//10
    TalonSRX rightMotor2 =  new TalonSRX(4);//10
    TalonSRX rightElevatorMotor = new TalonSRX(2);
	TalonSRX leftElevatorMotor = new TalonSRX(1);
    TalonSRX scalar = new TalonSRX(0);
    TalonSRX intakeBags = new TalonSRX(8);
	Joystick joystick = new Joystick(0);
	Joystick rightJoy = new Joystick(1);
	Joystick operator = new Joystick(2);
	Solenoid Solenoid2 = new Solenoid(2);//1
	Solenoid Solenoid1 = new Solenoid(1);
	// Anand's Solenoids, 0 is used for clamping, 3 is used for extending
	Solenoid Solenoid0 = new Solenoid(0);
	boolean clamped = false;
	Solenoid Solenoid3 = new Solenoid(3);
	boolean extended = false;
	Solenoid Solenoid5 = new Solenoid(5);
	//Solenoids for gear shifting
	Solenoid Solenoid4 = new Solenoid(4);//1
    Compressor compressor;
    boolean lowgear = false;
    int rightEncoder = rightMotor1.getSelectedSensorPosition(0);
    int leftEncoder = leftMotor1.getSelectedSensorPosition(0);
    double  leftspeed = 0;
	double rightspeed = 0;
	long idealright = 0;
	long idealleft = 0;
	int Counter = 0;
	  private static final String kDefaultAuto = "Default";
	    private static final String kCustomAuto = "My Auto";
	    private String m_autoSelected;
	    private SendableChooser<String> m_chooser = new SendableChooser<>();
	    int _loops=0; // this is a temporary variable used to count off dashboard updates later
	    
	    //Let's define our constants here.  Why?  Because they're easy to find when tuning.
	    //They are implemented below
	    private static final double Kp=4; 
	    private static final double Ki=0.04; // .01 * P 
	    private static final double Kf=0.0;  //no feed-forward on positon control
	    private static final double Kd=350; 
	    private static final int IZone= 10; 
    
    @Override
    public void robotInit() {
    	CameraServer.getInstance().startAutomaticCapture();
    	compressor = new Compressor(0);

   	
   	 m_chooser.addDefault("Default Auto", kDefaultAuto);
     m_chooser.addObject("My Auto", kCustomAuto);
     SmartDashboard.putData("Auto choices", m_chooser);

     //Configuration stuff.
     
     rightMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,0);    

     /*************************************************************************
      * Ramps.  The argument is how many seconds it takes to go from 0 to 100%
      * during voltage control.  There is a ramp for both open loop and closed 
      * (controlled) loop
      */
     rightMotor1.configOpenloopRamp(0.2, 0);
     rightMotor1.configClosedloopRamp(0.2, 0);
     
     // Talon setup.  Setting up + and - output for max and nominal.
     //These should never really change
     rightMotor1.configNominalOutputForward(0, 0);
     rightMotor1.configNominalOutputReverse(0, 0);
     rightMotor1.configPeakOutputForward(1,0);
     rightMotor1.configPeakOutputReverse(-1, 0);
     
     //set this if your encoder is hooked up backwards (A<--->B)
     //mine is backwards, so I've set it to true
     rightMotor1.setSensorPhase(true);  
     
     //implementation of tuning constants.  First argument is PID slot.  We're only
     //using slot 0.  If we had multiple tuning constants for the same controller
     // for different purposes, we can also have up to 3 slots pre-programmed
     rightMotor1.config_kF(0, Kf, 0); //No feed-forward on position control
     rightMotor1.config_kP(0, Kp, 0); //P factor!
     rightMotor1.config_kI(0, Ki, 0);
     rightMotor1.config_kD(0, Kd, 0); //note- these are defined above so they're easy to find and change.
     
     //one last term- configIntegralZone
     rightMotor1.config_IntegralZone(0, IZone, 0);
     
     leftMotor1.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0,0);    

     /*************************************************************************
      * Ramps.  The argument is how many seconds it takes to go from 0 to 100%
      * during voltage control.  There is a ramp for both open loop and closed 
      * (controlled) loop
      */
     leftMotor1.configOpenloopRamp(0.2, 0);
     leftMotor1.configClosedloopRamp(0.2, 0);
     
     // Talon setup.  Setting up + and - output for max and nominal.
     //These should never really change
     leftMotor1.configNominalOutputForward(0, 0);
     leftMotor1.configNominalOutputReverse(0, 0);
     leftMotor1.configPeakOutputForward(1,0);
     leftMotor1.configPeakOutputReverse(-1, 0);
     
     //set this if your encoder is hooked up backwards (A<--->B)
     //mine is backwards, so I've set it to true
     leftMotor1.setSensorPhase(false);  
     
     //implementation of tuning constants.  First argument is PID slot.  We're only
     //using slot 0.  If we had multiple tuning constants for the same controller
     // for different purposes, we can also have up to 3 slots pre-programmed
     leftMotor1.config_kF(0, Kf, 0); //No feed-forward on position control
     leftMotor1.config_kP(0, Kp, 0); //P factor!
     leftMotor1.config_kI(0, Ki, 0);
     leftMotor1.config_kD(0, Kd, 0); //note- these are defined above so they're easy to find and change.
     
     //one last term- configIntegralZone
     leftMotor1.config_IntegralZone(0, IZone, 0);
     
     
    }

    
    @Override
    public void autonomousInit() {
  
   	    }  
    public void autonomousPeriodic() {
    	
    }
  	 
    
    @Override
    public void teleopInit() {
 rightMotor1.setSelectedSensorPosition(0,0,0);
 leftMotor1.setSelectedSensorPosition(0,0,0);
 
   		 }    

    public void teleopPeriodic() {		
    	leftEncoder = (leftMotor1.getSelectedSensorPosition(0));
    	rightEncoder = (rightMotor1.getSelectedSensorPosition(0));
    	int change = (leftEncoder) - (rightEncoder);
    	 double targetPosition=joystick.getZ()*4096*10;
    	System.out.println(targetPosition); 
        if (joystick.getTrigger())
        {
        	if (change >= 100) {
        		rightMotor1.set(ControlMode.Position, targetPosition);
        		leftMotor1.set(ControlMode.Position, .9 * -targetPosition);
        	}
        	else if (change <= -100) {
        		rightMotor1.set(ControlMode.Position, .7 * targetPosition);
        		leftMotor1.set(ControlMode.Position, -targetPosition);        		
        	}
        	else {
        		rightMotor1.set(ControlMode.Position, targetPosition);
        		leftMotor1.set(ControlMode.Position, -targetPosition);
        	}
        	
            if (++_loops >= 10) {
                _loops = 0;
            SmartDashboard.putNumber("Trigger hit", targetPosition);
            SmartDashboard.putNumber("Z value:",joystick.getZ());
            }

            
        }
        //reguluar teleop.  Just give the motor power wherever you have the joystick set
        else if (!joystick.getRawButton(1)) {
            double axis=joystick.getY();
            rightMotor1.set(ControlMode.PercentOutput, axis);
            
            if (++_loops >= 10) {
                _loops = 0;
            SmartDashboard.putNumber("Trigger hit", targetPosition);
            }
        }
        SmartDashboard.putNumber("Current Pos Right:", rightMotor1.getSelectedSensorPosition(0));
        SmartDashboard.putNumber("Current Pos Left:", leftMotor1.getSelectedSensorPosition(0));
    }

    /**
     * This function is called periodically during test mode.
     */
    
   //Output Encoder Values, joystick values, and velocity in encoder pulses per 100msec
    		

    @Override
    public void testInit() {    
  

    }


    public void testPeriodic() {
    	leftElevatorMotor.set(ControlMode.PercentOutput, -.4);
		rightElevatorMotor.set(ControlMode.PercentOutput, .4);		 
    	


    }


}

