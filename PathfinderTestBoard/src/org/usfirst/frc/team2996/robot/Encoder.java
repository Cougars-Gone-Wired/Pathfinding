package org.usfirst.frc.team2996.robot;

import com.ctre.phoenix.motorcontrol.SensorCollection;

public class Encoder {

	private SensorCollection sensors;
	
	static double WHEEL_RADIUS = 3;
	static double CIRCUMFRENCE = 2 * Math.PI * WHEEL_RADIUS;
	static double PULSES_PER_REVOLUTION = 1440;
	static double distancePerPulse = CIRCUMFRENCE / PULSES_PER_REVOLUTION; // about .0131
	
	public Encoder(Robot robot) {
		sensors = robot.getSensors();
	}
	
	public int getCount() {
		return -sensors.getQuadraturePosition(); // sensorPhase makes the values that the talons see negative, not the values that we see, so these need to be made negative if sensorPhase is true
	}
	
	public double getDistanceInches() {
		return getCount() * distancePerPulse;
	}
	
	public double getDistanceFeet() {
		return (getCount() * distancePerPulse) / 12;
	}
	
	public void reset() {
		sensors.setQuadraturePosition(0, 10);
	}
}
