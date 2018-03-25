package org.team1218.lib.trajectory.io.main;

import java.io.File;
import java.io.IOException;

import org.team1218.lib.trajectory.io.PathManager;
import org.team1218.lib.trajectory.io.PathManager.PathPack;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.TrajectoryGenerator;
import com.team254.lib.trajectory.WaypointSequence;
import com.team254.lib.trajectory.TrajectoryGenerator.Config;

public class VulcanPathManagerTest {

	public static void main(String[] args) throws IOException {
		
		TrajectoryGenerator.Config trajConfig = new TrajectoryGenerator.Config();
		trajConfig.dt = .1;			// the time in seconds between each generated segment
		trajConfig.max_acc = 7.0;		// maximum acceleration for the trajectory, ft/s
		trajConfig.max_jerk = 7.0;	// maximum jerk (derivative of acceleration), ft/s
		trajConfig.max_vel = 7.0;// maximum velocity you want the robot to reach for this trajectory, ft/s
		
		WaypointSequence rightStartRightScaleWaypoints = new WaypointSequence(10);
	    rightStartRightScaleWaypoints.addWaypoint(new WaypointSequence.Waypoint(0.0,0.0,0.0));
	    rightStartRightScaleWaypoints.addWaypoint(new WaypointSequence.Waypoint(9, 0, 0));
	    rightStartRightScaleWaypoints.addWaypoint(new WaypointSequence.Waypoint(17.83,16,Math.toRadians(89.0)));
	    
	    PathManager.setPathPrefix("path/");
		PathManager.getPath(rightStartRightScaleWaypoints,trajConfig,2.0, "test");
	}

}
