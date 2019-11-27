package it.links.pert.codegen.scxml;

import java.io.File;

public class SCXML2RosSimulationMode extends SCXML2RosGenerationMode {

	public SCXML2RosSimulationMode(SCXML2RosGenerator generator) {
		super(generator);
	}

	@Override
	public boolean generate() {
		boolean success = false;
		success = createNewROSPackage() && super.generator.generateFSMBehavior();
		return success;
	}

	/**
	 * Create a new minimal catkin ROS package
	 * 
	 * @return true if the package is correctly created
	 */
	protected boolean createNewROSPackage() {
		final String worldPath = generator.getOutputDir() + generator.getLastGeneratedPkgName() + "/world";
		final File worldFile = new File(worldPath);
		return super.generator.createNewROSPackage() && worldFile.mkdir();
	}
}
