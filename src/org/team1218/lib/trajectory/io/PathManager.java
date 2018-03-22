package org.team1218.lib.trajectory.io;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.io.Writer;
import java.util.Date;

import com.team254.lib.trajectory.Path;
import com.team254.lib.trajectory.PathGenerator;
import com.team254.lib.trajectory.Trajectory;
import com.team254.lib.trajectory.TrajectoryGenerator.Config;
import com.team254.lib.trajectory.WaypointSequence;
import com.team254.lib.trajectory.Trajectory.Pair;
import com.team254.lib.trajectory.Trajectory.Segment;
import com.team254.lib.trajectory.WaypointSequence.Waypoint;

public class PathManager {
	
	public static class PathPack{
		public Path path;
		public Config config;
		public WaypointSequence ws;
		public double trackWidth;
	}
	
	public static Path getPath(WaypointSequence ws, Config config, double trackWidth, String name) {
		PathPack pathPack = new PathPack();
		pathPack.ws = ws;
		pathPack.config = config;
		pathPack.trackWidth = trackWidth;
		return getPath(pathPack,name);
	}
	
	public static Path getPath(PathPack input, String name) {
		File file = new File("paths/" + name + ".path");
		System.out.println("attempting to find path file.");
		try{
			PathPack output = readPathFile(file);
			if(output != null) {
				if(comparePathPacks(input,output)) {
					System.out.print("Path Match! Returning path from file.");
					return output.path;
				}else {
					System.out.println("Path Mismatch!");
				}
			}else {
				System.out.println("File invalid.");
			}
		}catch(IOException e){
			System.out.println("File not found.");
		}
		System.out.println("Failed to load path from file, generating path.");
		Path path = PathGenerator.makePath(input.ws, input.config, input.trackWidth, name);
		PathPack output = new PathPack();
		output.ws = input.ws;
		output.config = input.config;
		output.trackWidth = input.trackWidth;
		output.path = path;
		System.out.println("Done, attempting to write result to file.");
		try {
			creatPathFile(file,output);
		}catch(IOException e) {
			e.printStackTrace();
		}
		return output.path;
		
	}
	
	
	public static void creatPathFile(File file,PathPack pathPack) throws IOException {
		if(!file.exists()) {
			file.getParentFile().mkdirs();
			file.createNewFile();
		}
		Writer writer = new BufferedWriter(new FileWriter(file));
		
		Path path = pathPack.path;
		Config config = pathPack.config;
		WaypointSequence ws = pathPack.ws;
		double trackWidth = pathPack.trackWidth;
		
		//write comment block
		writer.write("# " + path.getName()+"\n");
		writer.write("# " + new Date() + "\n");
		writer.write("#\n");
		
		//Begin configuration block;
		writer.write("#Config: dt max_vel max_acc max_jerk track_width\n");
		writer.write(String.format(
				"%.3f %.3f %.3f %.3f %.3f\n",
				config.dt,config.max_vel,
				config.max_acc,config.max_jerk,
				trackWidth));
		writer.write("\n");
		//End configuration block
		
		//Begin Waypoint Block;
		writer.write("#Waypoints: x y theta\n");
		for(int i = 0; i < ws.getNumWaypoints(); i++) {
			Waypoint waypoint = ws.getWaypoint(i);
			writer.write(String.format(
					"%.3f %.3f %.3f\n",
					waypoint.x,waypoint.y,
					waypoint.theta));
		}
		writer.write("\n");
		//End Waypoint Block
		
		//Begine Path Block
		writer.write("#Path name numSegments\n");
		writer.write(path.getName() + "\n");
		path.goLeft();
		writer.write(path.getLeftWheelTrajectory().getNumSegments() + "\n");
		writer.write("#LeftTraj: pos vel acc jerk heading dt x y\n");
		for (int i = 0; i < path.getLeftWheelTrajectory().getNumSegments(); ++i) {
		     Segment segment = path.getLeftWheelTrajectory().getSegment(i);
		     writer.write(String.format(
            "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", 
            segment.pos, segment.vel, segment.acc, segment.jerk,
            segment.heading, segment.dt, segment.x, segment.y));
		}
		writer.write("#RightTraj: pos vel acc jerk heading dt x y\n");
		for (int i = 0; i < path.getRightWheelTrajectory().getNumSegments(); ++i) {
		     Segment segment = path.getRightWheelTrajectory().getSegment(i);
		     writer.write(String.format(
           "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", 
           segment.pos, segment.vel, segment.acc, segment.jerk,
           segment.heading, segment.dt, segment.x, segment.y));
		}
		writer.close();
	}
	
	public static PathPack readPathFile(File file) throws IOException {
		BufferedReader reader = new BufferedReader(new FileReader(file));
		String line;
		PathPack pathPack = new PathPack();
		pathPack.ws = new WaypointSequence(10);
		pathPack.path = null;
		String name = "";
		Trajectory leftTraj = null;
		Trajectory rightTraj = null;
		int segmentCounter = 0;
		int state = 0; //stateMachine 0: config 1:ws 2:traj
		int pathState = 0; //pathstateMachine 0:name 1:numPoints 2:leftTraj 3:rightTraj
		while((line = reader.readLine()) != null) {
			if(!line.startsWith("#")) {
				String tokens[] = line.split(" ");
				switch(state) {
				
				case 0:{
					if(tokens.length == 5) {
						Config config = new Config();
						config.dt = Double.parseDouble(tokens[0]);
						config.max_vel = Double.parseDouble(tokens[1]);
						config.max_acc = Double.parseDouble(tokens[2]);
						config.max_jerk = Double.parseDouble(tokens[3]);
						pathPack.config = config;
						pathPack.trackWidth = Double.parseDouble(tokens[4]);
					}else if(tokens.length == 1 && tokens[0].equals("")) {
						if(pathPack.config == null) {
							return null;
						}
						state++;
					}
					break;
				}
				
				case 1:{
					if(tokens.length == 3) {
						double x = Double.parseDouble(tokens[0]);
						double y = Double.parseDouble(tokens[1]);
						double theta = Double.parseDouble(tokens[2]);
						pathPack.ws.addWaypoint(new Waypoint(x,y,theta));
					}else if(tokens.length == 1 && tokens[0].equals("")){
						if(pathPack.ws.getNumWaypoints() < 2) {
							return null;
						}
						state++;
					}
					break;
				}
				case 2:{
					switch(pathState) {
					case 0:{
						if(tokens.length > 0) {
							name = tokens[0];
							pathState++;
						}else {
							return null;
						}
						break;
					}
					case 1:{
						if(tokens.length > 0) {
							int numSegments = Integer.parseUnsignedInt(tokens[0]);
							leftTraj = new Trajectory(numSegments);
							rightTraj = new Trajectory(numSegments);
							pathState++;
						}else {
							return null;
						}
						break;
					}
					case 2:{
						if(segmentCounter < leftTraj.getNumSegments()) {
							if(tokens.length == 8) {
								Segment seg = new Segment();
								seg.pos = Double.parseDouble(tokens[0]);
								seg.vel = Double.parseDouble(tokens[1]);
								seg.acc = Double.parseDouble(tokens[2]);
								seg.jerk = Double.parseDouble(tokens[3]);
								seg.heading = Double.parseDouble(tokens[4]);
								seg.dt = Double.parseDouble(tokens[5]);
								seg.x = Double.parseDouble(tokens[6]);
								seg.y = Double.parseDouble(tokens[7]);
								leftTraj.setSegment(segmentCounter, seg);
								segmentCounter ++;
							}else {
								return null;
							}
							break;
						}else {
							pathState ++;
							segmentCounter = 0;
						}
					}
					case 3:{
						if(segmentCounter < rightTraj.getNumSegments()) {
							if(tokens.length == 8) {
								Segment seg = new Segment();
								seg.pos = Double.parseDouble(tokens[0]);
								seg.vel = Double.parseDouble(tokens[1]);
								seg.acc = Double.parseDouble(tokens[2]);
								seg.jerk = Double.parseDouble(tokens[3]);
								seg.heading = Double.parseDouble(tokens[4]);
								seg.dt = Double.parseDouble(tokens[5]);
								seg.x = Double.parseDouble(tokens[6]);
								seg.y = Double.parseDouble(tokens[7]);
								rightTraj.setSegment(segmentCounter, seg);
								segmentCounter ++;
							}else {
								return null;
							}
						}
						if(segmentCounter == (rightTraj.getNumSegments())) {
							pathPack.path = new Path(name, new Pair(leftTraj,rightTraj));
						}
					}
					}
					break;
				}
				}
			}
		}
		return pathPack;
	}
	
	public static boolean compareConfig(Config a, Config b) {
		return (Math.abs(a.dt - b.dt) < 0.001) && (Math.abs(a.max_vel - b.max_vel) < 0.001) && (Math.abs(a.max_acc - b.max_acc) < 0.001) && (Math.abs(a.max_jerk - b.max_jerk) < 0.001);
	}
	
	public static boolean compareTrackWidth(double a, double b) {
		return (Math.abs(a - b) < 0.001);
	}
	
	public static boolean compareWaypoints(Waypoint a, Waypoint b) {
		return (Math.abs(a.x - b.x) < 0.001)&&(Math.abs(a.y - b.y) < 0.001) &&(Math.abs(a.theta - b.theta) < 0.001);
	}
	
	public static boolean compareWaypointSequences(WaypointSequence a, WaypointSequence b) {
		boolean equal = true;
		if(a.getNumWaypoints() != b.getNumWaypoints()) {
			System.out.println();
			return false;
		}
		for(int i = 0; i < a.getNumWaypoints(); i++) {
			if(compareWaypoints(a.getWaypoint(i),b.getWaypoint(i))== false) {
				equal = false;
			}
		}
		return equal;
	}
	
	public static boolean comparePathPacks(PathPack a, PathPack b) {
		return compareConfig(a.config,b.config) && compareTrackWidth(a.trackWidth,b.trackWidth) && compareWaypointSequences(a.ws,b.ws);
	}
}
