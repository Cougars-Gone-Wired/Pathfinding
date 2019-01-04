package org.usfirst.frc.team2996.robot;

import com.ctre.phoenix.motorcontrol.SensorCollection;

public class Encoders {

	public SensorCollection leftSensors;
	public SensorCollection rightSensors;
	
	static double WHEEL_RADIUS = 3;
	static double CIRCUMFRENCE = 2 * Math.PI * WHEEL_RADIUS;
	static double PULSES_PER_REVOLUTION = 1440;
	static double distancePerPulse = CIRCUMFRENCE / PULSES_PER_REVOLUTION;
	
	public Encoders(Robot robot) {
		leftSensors = robot.getLeftSensors();
		rightSensors = robot.getRightSensors();
	}
	
	
	public int getLeftCount() {
		return leftSensors.getQuadraturePosition();
	}
	public int getRightCount() {
		return -rightSensors.getQuadraturePosition();
	}
	public int getAverageCount() {
		return (getLeftCount() + getRightCount()) / 2;
	}
	
	
	public double getLeftDistanceInches() {
		return getLeftCount() * distancePerPulse;
	}
	public double getRightDistanceInches() {
		return getRightCount() * distancePerPulse;
	}
	public double getAverageDistanceInches() {
		return (getLeftDistanceInches() + getRightDistanceInches()) / 2;
	}
	
	
	public double getLeftDistanceFeet() {
		return (getLeftCount() * distancePerPulse) / 12;
	}
	public double getRightDistanceFeet() {
		return (getRightCount() * distancePerPulse) / 12;
	}
	public double getAverageDistanceFeet() {
		return (getLeftDistanceFeet() + getRightDistanceFeet()) / 2;
	}
	
	
	public void resetLeftEncoder() {
		leftSensors.setQuadraturePosition(0, 10);
	}

	public void resetRightEncoder() {
		rightSensors.setQuadraturePosition(0, 10);
	}

	public void reset() {
		leftSensors.setQuadraturePosition(0, 10);
		rightSensors.setQuadraturePosition(0, 10);
	}
}
