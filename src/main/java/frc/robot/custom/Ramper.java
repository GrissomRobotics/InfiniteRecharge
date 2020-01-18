package frc.robot.custom;

public class Ramper{
	private double step = 0;
	private double lastSpeed = 0;
	public Ramper(double step){
		this.step=step;
	}
	public double ramp(double currentSpeed) {
		double output=0;
		
		
	 	if (lastSpeed<0 && currentSpeed>0) {
	 		output = lastSpeed+step;
	 	} 
	 	else if (lastSpeed>0 && currentSpeed<0) {
	 		output = lastSpeed-step;
	 	} 
	 	else if (lastSpeed >= 0 && currentSpeed>lastSpeed) {
	 		output = lastSpeed + Math.min(step, currentSpeed-lastSpeed);
	 	} 
	 	else if (lastSpeed <= 0 && currentSpeed<lastSpeed) {
	 		output = lastSpeed - Math.min(step, lastSpeed-currentSpeed);
	 	} 
	 	else{
	 		output = currentSpeed;
	 	}
	 	lastSpeed = output;
	 	return output;
	}
	
	public void reset(){
		lastSpeed = 0;
	}
	
}