/*
 * Odometer.java
 */

package ev3Odometer;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.MotorPort;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta, thetaDeg;

	// constant
	public static double WB = Lab2.TRACK;
	public static double WR = Lab2.WHEEL_RADIUS;

	// variables
	public static int lastTachoL;// Tacho L at last sample
	public static int lastTachoR;// Tacho R at last sample
	public static int nowTachoL;// Current tacho L
	public static int nowTachoR;// Current tacho R

	// Resources
	static TextLCD t = LocalEV3.get().getTextLCD();
	private EV3LargeRegulatedMotor leftMotor; // L
	private EV3LargeRegulatedMotor rightMotor; // L

	// odometer update period, in ms
	private static final long ODOMETER_PERIOD = 25;

	// lock object for mutual exclusion
	private Object lock;

	// default constructor
	public Odometer(EV3LargeRegulatedMotor leftmotor,
			EV3LargeRegulatedMotor rightmotor) {
		x = 0.0;
		y = 0.0;
		theta = 0.0;
		lock = new Object();
		this.leftMotor = leftmotor;
		this.rightMotor = rightmotor;

	}

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;
		double distL, distR, deltaD, deltaT, dX, dY;

		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here

			nowTachoL = leftMotor.getTachoCount(); // get tacho counts
			nowTachoR = rightMotor.getTachoCount();
			distL = Math.PI * WR * (nowTachoL - lastTachoL) / 180; // compute
																	// wheel
			distR = Math.PI * WR * (nowTachoR - lastTachoR) / 180; // displacements
			lastTachoL = nowTachoL; // save tacho counts for next iteration
			lastTachoR = nowTachoR;
			deltaD = 0.5 * (distL + distR); // compute vehicle displacement
			deltaT = (distL - distR) / WB; // compute change in heading

			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				
				theta += deltaT;
				if (theta >= Math.PI*2) {
					theta = theta - Math.PI*2;
				}
				dX = deltaD * Math.sin(theta); // compute X component of
												// displacement
				dY = deltaD * Math.cos(theta); // compute Y component of
												// displacement
				x = x + dX;
				y = y + dY;
				thetaDeg = theta * 180/Math.PI;
			}

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}

	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = thetaDeg;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
}