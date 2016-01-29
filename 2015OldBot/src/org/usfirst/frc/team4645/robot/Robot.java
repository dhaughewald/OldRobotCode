//The code and documentation are both in their beginning stages and this may or may not be representative of the final
//version of the code
package org.usfirst.frc.team4645.robot;

//import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
//import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Ultrasonic; 
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.AnalogInput;



//Vision Processing
import java.lang.Math;
import java.util.Comparator;
import java.util.Vector;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ImageType;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SampleRobot;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot { 
	DigitalOutput custom1;
	RobotDrive rb;
    Joystick joy1, joy2 ; 
    CANTalon frontLeft, frontRight, backLeft, backRight;
    CANTalon flipMotor;
    CANTalon noodleMotor1, noodleMotor2;
    CANTalon liftMotor1, liftMotor2;
    PowerDistributionPanel pdp ;
    //Gyro gyro;
    //Encoder liftEncode;
    Ultrasonic rangeSense1; 
    Ultrasonic rangeSense2;
    Compressor compressor;
    Solenoid jeff;
    Timer timer;
    AnalogInput sanic;
    
   // int session;
   // Image frame;
    
    public class ParticleReport implements Comparator<ParticleReport>, Comparable<ParticleReport>{
			double PercentAreaToImageArea;
			double Area;
			double ConvexHullArea;
			double BoundingRectLeft;
			double BoundingRectTop;
			double BoundingRectRight;
			double BoundingRectBottom;
			
			public int compareTo(ParticleReport r)
			{
				return (int)(r.Area - this.Area);
			}
			
			public int compare(ParticleReport r1, ParticleReport r2)
			{
				return (int)(r1.Area - r2.Area);
			}
		};

		//Structure to represent the scores for the various tests used for target identification
		public class Scores {
			double Trapezoid;
			double LongAspect;
			double ShortAspect;
			double AreaToConvexHullArea;
		};

		//Images
		Image frame;
		Image binaryFrame;
		int imaqError;

		//Constants
		NIVision.Range TOTE_HUE_RANGE = new NIVision.Range(24, 49);	//Default hue range for yellow tote(24,49)
		NIVision.Range TOTE_SAT_RANGE = new NIVision.Range(67, 255);	//Default saturation range for yellow tote(67,255)
		NIVision.Range TOTE_VAL_RANGE = new NIVision.Range(49, 255);	//Default value range for yellow tote(49, 255)
		double AREA_MINIMUM = 0.5; //Default Area minimum for particle as a percentage of total image area
		double LONG_RATIO = 2.22; //Tote long side = 26.9 / Tote height = 12.1 = 2.22
		double SHORT_RATIO = 1.4; //Tote short side = 16.9 / Tote height = 12.1 = 1.4
		double SCORE_MIN = 75.0;  //Minimum score to be considered a tote
		double VIEW_ANGLE = 49.4; //View angle fo camera, set to Axis m1011 by default, 64 for m1013, 51.7 for 206, 52 for HD3000 square, 60 for HD3000 640x480
		NIVision.ParticleFilterCriteria2 criteria[] = new NIVision.ParticleFilterCriteria2[1];
		NIVision.ParticleFilterOptions2 filterOptions = new NIVision.ParticleFilterOptions2(0,0,1,1);
		Scores scores = new Scores();
		int session;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	 
    	SmartDashboard.putString("MyoVal:", "   ");

     	frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
 		binaryFrame = NIVision.imaqCreateImage(ImageType.IMAGE_U8, 0);
 		criteria[0] = new NIVision.ParticleFilterCriteria2(NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA, AREA_MINIMUM, 100.0, 0, 0);

 		
 		session = NIVision.IMAQdxOpenCamera("cam0",
                 NIVision.IMAQdxCameraControlMode.CameraControlModeController);
         NIVision.IMAQdxConfigureGrab(session);
     	//custom1 = new DigitalOutput(17);
    	
    	custom1 = new DigitalOutput(17);
    	
		joy1 = new Joystick(1);
		joy2 = new Joystick(2);
	
		frontLeft = new CANTalon(0);
		frontRight = new CANTalon(3);
		backLeft = new CANTalon(1);
		backRight = new CANTalon(7);
		
		liftMotor1 = new CANTalon(2);
		liftMotor2 = new CANTalon(6);
		//flipMotor = new CANTalon(6);
		noodleMotor1 = new CANTalon(8);//4
		noodleMotor2 = new CANTalon(5);//5
		//gyro = new Gyro(0);
		timer = new Timer();
		sanic = new AnalogInput(1);
		
		//liftEncode = new Encoder(0, 1);
		//liftEncode = new Encoder(0, 0, true, Encoder.EncodingType.k1X);
		
		rb = new RobotDrive(frontLeft, backLeft, frontRight, backRight);
		pdp = new PowerDistributionPanel() ;
		
		//pneumatics
		compressor = new Compressor(0); //create a compressor with the default pcmid.
    	compressor.setClosedLoopControl(true);
    	compressor.start();
    	jeff = new Solenoid(0,0);
    }
    
    public void autonomousInit() {
    	 timer.reset();
    	 
 		rb.setInvertedMotor(MotorType.kFrontRight, true);
 		rb.setInvertedMotor(MotorType.kRearRight, true);
     	timer.start();
     	while(true){
     		SmartDashboard.putNumber("Time: ", timer.get());
     		while(timer.get() > 0.0 && timer.get() < 0.5){
     			jeff.set(true);
     			rb.mecanumDrive_Cartesian( 0.00, -.30, 0.00,  0.00);
     		}
     		while(timer.get() > .7 && timer.get() < 1.50){
     			jeff.set(false);
     			timer.delay(.7);
     			liftMotor1.set(.5);
     			liftMotor2.set(-.5);
     		}
     		while(timer.get() > 1.5 && timer.get() < 6.00){
     	 		rb.setInvertedMotor(MotorType.kFrontRight, true);
     	 		rb.setInvertedMotor(MotorType.kRearRight, true);
     			if(sanic.getVoltage() < .5){	
         			rb.mecanumDrive_Cartesian(0.00, -0.30, .02,  0.00);}
         		if(sanic.getVoltage() > .5){
         			rb.mecanumDrive_Cartesian(0.00, -0.30, -.02, 0.00);}
     		}
     		if(timer.get() > 6.0 && timer.get() < 7.00){
     			timer.stop();
     			return;
     		}else
     		if(timer.get() > 5.0 && timer.get() < 6.00){
     			
     		}
     	}
    }

    public void teleopInit() {
    	pdp.clearStickyFaults();
    	
    	boolean isStacked = false;
    	boolean flipping = false;
    	boolean flipClock = true;
    	int noodling = 0;
    	int liftLvl = 0;
    	int desiredLvl = 0;
    	String myo = "z";
    	
    	double lastEncode = 0;
    	double encodeMag = 0;
    	double rate = 0;
    	double liftMag = 0;
		double tempYMag = 0;
		double tempXMag = 0;
		double tempTwist= 0;
		
		NIVision.IMAQdxStartAcquisition(session);
    	while( isOperatorControl() && isEnabled() )
    	{
    		liftMotor1.set(0.00);
    		liftMotor2.set(0.00);
    		
    		NIVision.IMAQdxGrab(session, frame, 1);
            CameraServer.getInstance().setImage(frame);
            		
    		//sets the Speed Constants for each direction based on the 4th axis setting
    		//double turnSpeed = (joy1.getRawAxis(3) + 2)*(joy1.getRawAxis(3) + 2) ; //
    		//double YSpeed = (joy1.getRawAxis(3) + 2)*(joy1.getRawAxis(3) + 2) ;    //  general speed controlled by 4th axis
    		//double XSpeed = (joy1.getRawAxis(3) + 2)*(joy1.getRawAxis(3) + 2) ;    // (why * 1.5?) ---> ***NEEDS TESTING
    		
    		double turnSpeed = joy1.getRawAxis(3) + 2;
    		double YSpeed = joy1.getRawAxis(3) + 2;
    		double XSpeed = joy1.getRawAxis(3) + 2;

    		double liftSpeed = (1 / (joy2.getRawAxis(3)+2));
    		
    		
    		//if the robot is carrying a stack of totes, move with gradual acceleration
    		if(!isStacked){
    			tempYMag = joy1.getY(); 
    			tempXMag = joy1.getX();
    			tempTwist = joy1.getTwist();
    		}
    		else{
    			if(tempYMag < joy1.getY()){tempYMag = tempYMag + 0.0005;}
    			else{tempYMag = tempYMag - 0.001 ;}
    			
    			if(tempXMag < joy1.getX()){tempXMag = tempXMag + 0.0005;}
    			else{tempXMag = tempXMag - 0.001;}
    			
    			if(tempTwist < joy1.getTwist()){tempTwist = tempTwist + 0.0005;}
    			else{tempTwist = tempTwist - 0.0005;}
    		}

    		if(joy1.getRawButton(2)){tempXMag  = 0;}//no strafe
    		if(joy1.getRawButton(3)){tempTwist = 0;}//no turn
    		if(joy1.getRawButton(1)){tempYMag  = 0;}//no forward
    		if(joy1.getRawButton(5)){isStacked = false;} //if button 5, its safe to accelerate normally 
    		if(joy1.getRawButton(6)){isStacked = true;}//if Button 6, we are holding totes and acceleration is dangerous
    		if(joy1.getRawButton(12)){
    			custom1.set(true);
    			custom1.pulse(17, 5);
    			}
    		else{
    		custom1.set(false);
    		}
    		
    		//noodle intake/output buttons
    		if(joy2.getRawButton(9)){noodling  = 0;} //turns off the noodle intake
    		if(joy2.getRawButton(8)){noodling  = 1;} //turns on the noodle intake
    		if(joy2.getRawButton(7)){noodling  = 2;} //turns on the noodle output
    		
    		if(joy2.getRawButton(11)){desiredLvl = 0;}//button moves lift to level 0 (~8 inches)
    		if(joy2.getRawButton(9)){desiredLvl  = 1;}//button to level 1(~1ft 2in)
    		if(joy2.getRawButton(7)){desiredLvl  = 2;}//button to level 2(~2ft 2in)
    		if(joy2.getRawButton(8)){desiredLvl  = 3;}//button to level 3(~3ft 2in)
    		if(joy2.getRawButton(10)){desiredLvl = 4;}//button to level 4(~4ft 2in)
    		
    		if(joy2.getRawButton(1)){jeff.set(true);} //fire solenoid
    		if(joy2.getRawButton(2)){jeff.set(false);} // retract solenoid
    		
    		//noodle intake motor setting
    		if(noodling == 0){noodleMotor1.set(0.0);}
    		if(noodling == 1){noodleMotor1.set(0.6);}
    		if(noodling == 2){noodleMotor1.set(-.6);}
    		if(noodling == 0){noodleMotor2.set(0.0);}
    		if(noodling == 1){noodleMotor2.set(0.6);}
    		if(noodling == 2){noodleMotor2.set(-.6);}
    		
    		
    		/**
    		rate = (joy2.getY() * 500); 
    		
    		encodeMag = liftEncode.getRate();
    		
    		if(lastEncode / encodeMag < 1.00){
    			encodeMag = 0;
    		}
    		
    		if(encodeMag < rate){
        		liftMag = liftMag + 0.01;
        		}
    		else if(encodeMag == 0){
    			liftMag = liftMag;
    			}
        	else if(encodeMag > rate){
        		liftMag = liftMag - 0.01;
        		}
        		
        		liftMotor1.set(liftMag);  
        		liftMotor2.set(liftMag* -1);  
        		
        		lastEncode = encodeMag ; 
        		
    		*/

    		if(joy2.getY() < 0){
    		liftMotor1.set(liftSpeed *  .3* joy2.getY()); //.3
    		liftMotor2.set(liftSpeed * -.3* joy2.getY()); //-.3
    		}
    		else if(joy2.getY() > 0){
    		liftMotor1.set(liftSpeed * .75* joy2.getY());  //.75
    		liftMotor2.set(liftSpeed * -.75* joy2.getY());  //-.75
    		} 
    		
    		rb.mecanumDrive_Cartesian( tempXMag/XSpeed, tempYMag/YSpeed, tempTwist/turnSpeed,  0.00);
    		rb.setInvertedMotor(MotorType.kFrontRight, true);
    		rb.setInvertedMotor(MotorType.kRearRight, true);
    		
    		//basic elevator control
    	/**if(joy2.getY() > 0){
    		liftMotor1.set(.05* joy2.getY());
    		liftMotor2.set(-.05* joy2.getY());
    		}
    		else if(joy2.getY() < 0){
    		liftMotor1.set(.75* joy2.getY());
    		liftMotor2.set(-.75* joy2.getY());
    		} */
    		
    		//smartDash live info display
    		//SmartDashboard.putBoolean("Flipping?", flipping);
    		//SmartDashboard.putNumber("Current Lift Level: ", liftLvl);
    		//SmartDashboard.putNumber("Desired Lift Level: ", desiredLvl);
    		//SmartDashboard.putBoolean("Carrying Stacks?", isStacked);
    		//SmartDashboard.putNumber("Joystick Y: ", tempYMag);
    		//SmartDashboard.putNumber("Joystick X: ", tempXMag);
    		//SmartDashboard.putNumber("Joystick Twist", tempTwist);
    //		SmartDashboard.putNumber("Gyro:  ", gyro.getRate());
    		SmartDashboard.putNumber("LIFTING: ", joy2.getY());
    		//SmartDashboard.putNumber("encoder: ", liftEncode.get());
    		//SmartDashboard.putNumber("Rate: ", liftEncode.getRate());
    		
    		SmartDashboard.putNumber("Speed: ", turnSpeed);
    		SmartDashboard.putNumber("UltraSonic: ", sanic.getVoltage());
    		
    	}
    	NIVision.IMAQdxStopAcquisition(session);
    }
    
    

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){
    	
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
    double ratioToScore(double ratio)
		{
			return (Math.max(0, Math.min(100*(1-Math.abs(1-ratio)), 100))); 
		}

		/**
		 * Method to score convex hull area. This scores how "complete" the particle is. Particles with large holes will score worse than a filled in shape
		 */
		double ConvexHullAreaScore(ParticleReport report)
		{
			return ratioToScore((report.Area/report.ConvexHullArea)*1.18);
		}

		/**
		 * Method to score if the particle appears to be a trapezoid. Compares the convex hull (filled in) area to the area of the bounding box.
		 * The expectation is that the convex hull area is about 95.4% of the bounding box area for an ideal tote.
		 */
		double TrapezoidScore(ParticleReport report)
		{
			return ratioToScore(report.ConvexHullArea/((report.BoundingRectRight-report.BoundingRectLeft)*(report.BoundingRectBottom-report.BoundingRectTop)*.954));
		}

		/**
		 * Method to score if the aspect ratio of the particle appears to match the long side of a tote.
		 */
		double LongSideScore(ParticleReport report)
		{
			return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop))/LONG_RATIO);
		}

		/**
		 * Method to score if the aspect ratio of the particle appears to match the short side of a tote.
		 */
		double ShortSideScore(ParticleReport report){
			return ratioToScore(((report.BoundingRectRight-report.BoundingRectLeft)/(report.BoundingRectBottom-report.BoundingRectTop))/SHORT_RATIO);
		}

		/**
		 * Computes the estimated distance to a target using the width of the particle in the image. 
		 *
		 * @param image The image to use for measuring the particle estimated rectangle
		 * @param report The Particle Analysis Report for the particle
		 * @param isLong Boolean indicating if the target is believed to be the long side of a tote
		 * @return The estimated distance to the target in feet.
		 */
		double computeDistance (Image image, ParticleReport report, boolean isLong) {
			double normalizedWidth, targetWidth;
			NIVision.GetImageSizeResult size;

			size = NIVision.imaqGetImageSize(image);
			normalizedWidth = 2*(report.BoundingRectRight - report.BoundingRectLeft)/size.width;
			targetWidth = isLong ? 26.0 : 16.9;

			return  targetWidth/(normalizedWidth*12*Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
		}
		void getInfo(){
			NIVision.IMAQdxStartAcquisition(session);
		
		
            NIVision.IMAQdxGrab(session, frame, 1);
            
            CameraServer.getInstance().setImage(frame);

			//Threshold the image looking for yellow (tote color)
			NIVision.imaqColorThreshold(binaryFrame, frame, 255, NIVision.ColorMode.HSV, TOTE_HUE_RANGE, TOTE_SAT_RANGE, TOTE_VAL_RANGE);

			//Send particle count to dashboard
			int numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			SmartDashboard.putNumber("Masked particles", numParticles);

			//Send masked image to dashboard to assist in tweaking mask.
			CameraServer.getInstance().setImage(binaryFrame);

			//filter out small particles
			float areaMin = (float)SmartDashboard.getNumber("Area min %", AREA_MINIMUM);
			criteria[0].lower = areaMin;
			imaqError = NIVision.imaqParticleFilter4(binaryFrame, binaryFrame, criteria, filterOptions, null);

			//Send particle count after filtering to dashboard
			numParticles = NIVision.imaqCountParticles(binaryFrame, 1);
			SmartDashboard.putNumber("Filtered particles", numParticles);

			if(numParticles > 0)
			{
				//Measure particles and sort by particle size
				Vector<ParticleReport> particles = new Vector<ParticleReport>();
				for(int particleIndex = 0; particleIndex < numParticles; particleIndex++)
				{
					ParticleReport par = new ParticleReport();
					par.PercentAreaToImageArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA_BY_IMAGE_AREA);
					par.Area = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_AREA);
					par.ConvexHullArea = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_CONVEX_HULL_AREA);
					par.BoundingRectTop = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_TOP);
					par.BoundingRectLeft = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_LEFT);
					par.BoundingRectBottom = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_BOTTOM);
					par.BoundingRectRight = NIVision.imaqMeasureParticle(binaryFrame, particleIndex, 0, NIVision.MeasurementType.MT_BOUNDING_RECT_RIGHT);
					particles.add(par);
				}
				particles.sort(null);

				//This example only scores the largest particle. Extending to score all particles and choosing the desired one is left as an exercise
				//for the reader. Note that the long and short side scores expect a single tote and will not work for a stack of 2 or more totes.
				//Modification of the code to accommodate 2 or more stacked totes is left as an exercise for the reader.
				scores.Trapezoid = TrapezoidScore(particles.elementAt(0));
				SmartDashboard.putNumber("Trapezoid", scores.Trapezoid);
				scores.LongAspect = LongSideScore(particles.elementAt(0));
				SmartDashboard.putNumber("Long Aspect", scores.LongAspect);
				scores.ShortAspect = ShortSideScore(particles.elementAt(0));
				SmartDashboard.putNumber("Short Aspect", scores.ShortAspect);
				scores.AreaToConvexHullArea = ConvexHullAreaScore(particles.elementAt(0));
				SmartDashboard.putNumber("Convex Hull Area", scores.AreaToConvexHullArea);
				boolean isTote = scores.Trapezoid > SCORE_MIN && (scores.LongAspect > SCORE_MIN || scores.ShortAspect > SCORE_MIN) && scores.AreaToConvexHullArea > SCORE_MIN;
				boolean isLong = scores.LongAspect > scores.ShortAspect;
				
				boolean isOptimalDistance = false;
				double dist = computeDistance(binaryFrame, particles.elementAt(0), isLong);
				if ( (dist <= 2.5) && (dist >= 1.6) )
				{
				isOptimalDistance = true;
				}

				//Send distance and tote status to dashboard. The bounding rect, particularly the horizontal center (left - right) may be useful for rotating/driving towards a tote
				SmartDashboard.putBoolean("IsTote", isTote);
				SmartDashboard.putNumber("Distance", dist);
				SmartDashboard.putBoolean("isOptimalPickup", isOptimalDistance);
				
			} else {
				SmartDashboard.putBoolean("IsTote", false);
			}
			
			Timer.delay(0.005);				// wait for a motor update time
		
		NIVision.IMAQdxStopAcquisition(session);
		}


    
}