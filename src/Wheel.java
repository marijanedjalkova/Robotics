
public class Wheel {
	private int radius;
	private int velocity;
	private int rotationRate;
	private boolean isLeft;
	
	public Wheel(int radius, boolean isLeft){
		this.radius = radius;
		this.isLeft = isLeft;
		velocity = 0;
		rotationRate = 0;
	}
	
	public void setVelocity(int v){
		velocity = v;
	}
	
	public void setRotationRate (int rr){
		rotationRate = rr;
	}
	
	public int getRadius(){
		return radius;
	}
	
	public int getVelocity(){
		return velocity;
	}
	
	public int getRotationRate(){
		return rotationRate;
	}
	
	public boolean isLeft(){
		return isLeft;
	}
	
}
