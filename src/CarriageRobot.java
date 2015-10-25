import java.util.List;

import geometry.IntPoint;
import renderables.Renderable;

public class CarriageRobot extends PotentialFieldsRobot {

	public CarriageRobot(String imagePath, IntPoint startingLocation, IntPoint goalLocation, int radius,
			int sensorRange, int sensorDensity, int goalRadius, List<Renderable> obstacles) {
		super(imagePath, startingLocation, goalLocation, radius, sensorRange, sensorDensity, goalRadius, obstacles);
			
	}
	
	protected void createPath(){
		
	}
	

}
