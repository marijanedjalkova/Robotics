import java.util.List;

import geometry.IntPoint;
import renderables.Renderable;

public class CarriageRobot extends PotentialFieldsRobot {
	private Wheel lWheel, rWheel; //two wheels of the robot
	private int width; //distance between the wheels

	public CarriageRobot(String imagePath, IntPoint startingLocation, IntPoint goalLocation, int radius,
			int sensorRange, int sensorDensity, int goalRadius, List<Renderable> obstacles) {
		super(imagePath, startingLocation, goalLocation, radius, sensorRange, sensorDensity, goalRadius, obstacles);
		// TODO Auto-generated constructor stub
		lWheel = new Wheel(1, true);
		rWheel = new Wheel(1, false);
		width = 2 * radius;
		
	}

}
