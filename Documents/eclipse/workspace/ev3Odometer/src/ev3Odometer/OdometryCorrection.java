/* 
 * OdometryCorrection.java
 */
package ev3Odometer;

import java.io.File;

import lejos.hardware.Sound;
import lejos.hardware.Sounds;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;

public class OdometryCorrection extends Thread {
	private static final long CORRECTION_PERIOD = 10;
	private Odometer odometer;
	EV3ColorSensor lightSensor = new EV3ColorSensor(SensorPort.S1);
	float[] sampleRed = {0};

	// constructor
	public OdometryCorrection(Odometer odometer) {
		this.odometer = odometer;
	}

	// run method (required for Thread)
	public void run() {
		long correctionStart, correctionEnd;

		while (true) {
			correctionStart = System.currentTimeMillis();

			// put your correction code here
			lightSensor.getRedMode().fetchSample(sampleRed, 0);
			if(((int)(sampleRed[0]*100)) < 35){
				Sound.beep();
			}
			
			
			// this ensure the odometry correction occurs only once every period
			correctionEnd = System.currentTimeMillis();
			if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
				try {
					Thread.sleep(CORRECTION_PERIOD
							- (correctionEnd - correctionStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometry correction will be
					// interrupted by another thread
				}
			}
		}
	}
}