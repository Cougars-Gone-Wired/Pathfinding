package org.usfirst.frc.team2996.robot;

import java.io.File;

import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Robot extends TimedRobot {

	static final int FRONT_LEFT_MOTOR_ID = 4;
	static final int REAR_LEFT_MOTOR_ID = 5;
	static final int FRONT_RIGHT_MOTOR_ID = 6;
	static final int REAR_RIGHT_MOTOR_ID = 7;
	
	WPI_TalonSRX masterLeftMotor, slaveLeftMotor, masterRightMotor, slaveRightMotor;
	SensorCollection leftSensors, rightSensors;
	AHRS gyro;
	Encoders encoders;
	
	File myFile = new File("myfile.csv");
	Trajectory trajectory = Pathfinder.readFromCSV(myFile);
	TankModifier modifier = new TankModifier(trajectory).modify(0.5);
	EncoderFollower left = new EncoderFollower(modifier.getLeftTrajectory());
	EncoderFollower right = new EncoderFollower(modifier.getRightTrajectory());
	
	double ticksPerRevolution = 1440;
	double wheelDiameter = 0.5; //ft
	double P = 3.3;
	double I = 0;
	double D = 0;
	double maxVelocity = 10; //fps
	double accelerationGain = 0;
	
	int segment = 0;
	
	@Override
	public void robotInit() {
		masterLeftMotor = new WPI_TalonSRX(FRONT_LEFT_MOTOR_ID);
	    slaveLeftMotor = new WPI_TalonSRX(REAR_LEFT_MOTOR_ID);
	    masterRightMotor = new WPI_TalonSRX(FRONT_RIGHT_MOTOR_ID);
	    slaveRightMotor = new WPI_TalonSRX(REAR_RIGHT_MOTOR_ID);
	    
	    masterLeftMotor.setInverted(true);
	    slaveLeftMotor.setInverted(true);
	    slaveLeftMotor.follow(masterLeftMotor);
	    slaveRightMotor.follow(masterRightMotor);
		
	    leftSensors = new SensorCollection(masterLeftMotor);
		rightSensors = new SensorCollection(masterRightMotor);
		
		gyro = new AHRS(SPI.Port.kMXP);
		
		encoders = new Encoders(this);
		
		left.configureEncoder(leftSensors.getQuadraturePosition(), 1440, 0.5);
		right.configureEncoder(rightSensors.getQuadraturePosition(), 1440, 0.5);
		left.configurePIDVA(P, I, D, 1 / maxVelocity, accelerationGain);
		right.configurePIDVA(P, I, D, 1 / maxVelocity, accelerationGain);
		
		setPeriod(0.05);
	}

	@Override
	public void autonomousInit() {
		
	}

	@Override
	public void autonomousPeriodic() {
		double l = left.calculate(encoders.getLeftCount());
		double r = right.calculate(encoders.getRightCount());
		
		double gyroHeading = gyro.getAngle();
		double desiredHeading = Pathfinder.r2d(left.getHeading());
		
		double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
		double turn = 0.8 * (-1.0 / 80.0) * angleDifference;
		
		masterLeftMotor.set(l + turn);
		masterRightMotor.set(r - turn);
		
		if (segment < trajectory.length()) {		
			Trajectory.Segment sourceSeg = trajectory.get(segment);
			Trajectory.Segment leftSeg = modifier.getLeftTrajectory().get(segment);
			Trajectory.Segment rightSeg = modifier.getRightTrajectory().get(segment);
			SmartDashboard.putNumber("Source Position", sourceSeg.position);
			SmartDashboard.putNumber("Left Position", leftSeg.position);
			SmartDashboard.putNumber("Right Position", rightSeg.position);
			SmartDashboard.putNumber("Source Velocity", sourceSeg.velocity);
			SmartDashboard.putNumber("Left Velocity", leftSeg.velocity);
			SmartDashboard.putNumber("Right Velocity", rightSeg.velocity);
			segment++;
		}
		SmartDashboard.putNumber("average encoder", encoders.getAverageCount());
		SmartDashboard.putNumber("left encoder", encoders.getLeftCount());
		SmartDashboard.putNumber("right encoder", encoders.getRightCount());
		SmartDashboard.putNumber("average feet", encoders.getAverageDistanceFeet());
		SmartDashboard.putNumber("left feet", encoders.getLeftDistanceFeet());
		SmartDashboard.putNumber("right feet", encoders.getRightDistanceFeet());
		SmartDashboard.putNumber("left output", l + turn);
		SmartDashboard.putNumber("right output", r - turn);
	}

	@Override
	public void teleopPeriodic() {
		
	}

	@Override 
	public void disabledInit() {
		encoders.reset();
		left.reset();
		right.reset();
		segment = 0;
	}
	
	@Override
	public void testPeriodic() {
		
	}

	public SensorCollection getLeftSensors() {
		return leftSensors;
	}

	public SensorCollection getRightSensors() {
		return rightSensors;
	}
	
}
