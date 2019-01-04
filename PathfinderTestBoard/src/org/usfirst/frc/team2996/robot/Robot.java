/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2996.robot;

import java.io.File;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;

public class Robot extends TimedRobot {

	private WPI_TalonSRX masterMotor;
	private WPI_TalonSRX slaveMotor;
	private SensorCollection sensors;
	private Encoder encoder;
	
	File myFile = new File("/home/lvuser/paths/JaciLow_source_Jaci.csv");
	Trajectory trajectory = Pathfinder.readFromCSV(myFile);
	EncoderFollower follower = new EncoderFollower(trajectory);
	
	double ticksPerRevolution = 1440;
	double wheelDiameter = 0.5; //ft
	double P = .55; //.55 low //.125 high
	double I = 0;
	double D = 0;
	double maxVelocity = 5; //fps //5 low //20 high
	double accelerationGain = 0;
	
	int segment = 0;
	
	@Override
	public void robotInit() {
		masterMotor = new WPI_TalonSRX(0);
		slaveMotor = new WPI_TalonSRX(1);
		slaveMotor.follow(masterMotor); 
		sensors = new SensorCollection(masterMotor);
		encoder = new Encoder(this);
		
		follower.configureEncoder(sensors.getQuadraturePosition(), 1440, 0.5);
		follower.configurePIDVA(P, I, D, 1 / maxVelocity, accelerationGain);
		
		setPeriod(0.05);
	}
	
	@Override
	public void autonomousInit() {
		
	}

	@Override
	public void autonomousPeriodic() {
		double output = follower.calculate(encoder.getCount());
		masterMotor.set(output);
		if (segment < trajectory.length()) {		
			Trajectory.Segment seg = trajectory.get(segment);
			SmartDashboard.putNumber("Position", seg.position);
			SmartDashboard.putNumber("Velocity", seg.velocity);
			segment++;
		}
		SmartDashboard.putNumber("encoder", encoder.getCount());
		SmartDashboard.putNumber("feet", encoder.getDistanceFeet());
		SmartDashboard.putNumber("output", output);
	}

	@Override
	public void teleopPeriodic() {
		
	}

	@Override 
	public void disabledInit() {
		encoder.reset();
		follower.reset();
		segment = 0;
	}
	
	@Override
	public void testPeriodic() {
		
	}

	public SensorCollection getSensors() {
		return sensors;
	}
	
}
