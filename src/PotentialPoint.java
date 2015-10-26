
public class PotentialPoint {
	public double x;
	public double y;
	public double heading;
	public double changeX, changeY;
	
	public PotentialPoint(double x, double y, double h, double changeX, double changeY){
		this.x = x;
		this.y = y;
		heading = h;
		this.changeX = changeX;
		this.changeY = changeY;
	}
}
