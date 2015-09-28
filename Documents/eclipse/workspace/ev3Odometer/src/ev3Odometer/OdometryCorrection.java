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

			int xLine = ((int)(Math.abs(odometer.getX())+30))/30;
			int yLine = ((int)(Math.abs(odometer.getY())+30))/30;
			double theta = odometer.getTheta();
			
			// put your correction code here
			lightSensor.getRedMode().fetchSample(sampleRed, 0);
			if(((int)(sampleRed[0]*100)) < 35){
				//going up -- -10 < theta < 10
				if(Math.PI*(-1.0/18)< theta && theta < Math.PI*(1.0/18)){
					Sound.beep();
					odometer.setY(15 + 30*(yLine-1));
				}
				//going right -- 80<theta<100
				if(Math.PI*(4.0/9) < theta && theta < Math.PI*(5.0/9)){
					Sound.buzz();
					odometer.setX(15+30*(xLine-1));
				}
				//going down -- 170<theta<190
				if(Math.PI*(17.0/18)<theta && theta < Math.PI*(19.0/18)){
					Sound.beep();
					odometer.setY(15 + 30*(yLine-1));
				}
				//going left -- 260<theta<280
				if(Math.PI*(13.0/9)<theta && theta < Math.PI*(14.0/9)){
					Sound.buzz();
					odometer.setX(15+30*(xLine-1));
				}
				else{
					//Sound.beep();
				}
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