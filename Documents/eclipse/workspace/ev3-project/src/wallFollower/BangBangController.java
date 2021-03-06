package wallFollower;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {
	private final int bandCenter, bandwidth;
	private final int motorLow, motorHigh;
	private int distance;
	private int errorFilter;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	public BangBangController(EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor, int bandCenter, int bandwidth,
			int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandwidth = bandwidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		leftMotor.setSpeed(motorHigh); // Start robot moving forward
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
		errorFilter = 0;
	}

	@Override
	public void processUSData(int distance) {
		this.distance = distance;
		// TODO: process a movement based on the us distance passed in
		// (BANG-BANG style)


		if (distance < (bandCenter)) {
			rightMotor.setSpeed(motorLow);
			leftMotor.setSpeed(motorHigh);
			leftMotor.forward();
			rightMotor.forward();
			errorFilter = 0;
		} else if (distance > (bandCenter + bandwidth)) {
			if (distance > 255 && errorFilter < 15){
				errorFilter++;
			}else if (distance <= 255){
				leftMotor.setSpeed(motorLow);
				rightMotor.setSpeed(motorHigh);
				leftMotor.forward();
				rightMotor.forward();
				errorFilter = 0;
			}
		} else {
			rightMotor.setSpeed(motorHigh);
			leftMotor.setSpeed(motorHigh);
			leftMotor.forward();
			rightMotor.forward();
			errorFilter = 0;
		}

	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
