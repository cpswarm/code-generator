package it.links.pert.codegen.scxml;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import javax.xml.stream.XMLStreamException;

import org.apache.commons.scxml2.io.SCXMLReader;
import org.apache.commons.scxml2.model.CustomAction;
import org.apache.commons.scxml2.model.ModelException;
import org.apache.commons.scxml2.model.SCXML;
import org.apache.velocity.Template;
import org.apache.velocity.VelocityContext;
import org.apache.velocity.app.VelocityEngine;
import org.apache.velocity.runtime.RuntimeConstants;
import org.apache.velocity.runtime.resource.loader.ClasspathResourceLoader;
import org.slf4j.LoggerFactory;
import org.slf4j.Logger;

import it.links.pert.codegen.generator.CodeGenerator;

public class SCXML2RosGenerator implements CodeGenerator {
	
	private static final Logger LOGGER = LoggerFactory.getLogger(SCXML2RosGenerator.class.getName());

	final static String FSM_TEMPLATE_FILE = "template/ros/state_machine.vm";
	final static String PACKAGE_XML_TEMPLATE_FILE = "template/ros/package_xml.vm";
	final static String CMAKELISTS_TEMPLATE_FILE = "template/ros/cmakelists.vm";
	final static String SMACH_FILE_NAME = "behaviour.py";
	final static String ROS_PKG_DEAFULT_NAME = "fsm_behaviour";

	private VelocityEngine ve;
	private Template template;
	private String inputPath;
	private String outputDir;
	private String rosPkgName;

	public SCXML2RosGenerator(String inputPath, String outputDir) {
		this.inputPath = inputPath;
		this.outputDir = outputDir;
		rosPkgName = ROS_PKG_DEAFULT_NAME;
		ve = new VelocityEngine();
		ve.setProperty(RuntimeConstants.RESOURCE_LOADER, "classpath");
		ve.setProperty("classpath.resource.loader.class", ClasspathResourceLoader.class.getName());
		template = ve.getTemplate(FSM_TEMPLATE_FILE);
		ve.setProperty("space.gobbling", "structured");
	}

	/**
	 * Create a new catkin package
	 * 
	 * @return true if everything went well
	 */
	boolean createROSPackage() {
		LOGGER.info("Creating ROS package");
		File newDirectory = new File(outputDir + rosPkgName);
		int count = 1;
		String tmpDir = null;
		// If this package name already exist add a number at the end
		while (newDirectory.exists()) {
			rosPkgName = ROS_PKG_DEAFULT_NAME + count++;
			tmpDir = outputDir + rosPkgName;
			newDirectory = new File(tmpDir);
		}
		File scriptsDirectory = new File(newDirectory, "scripts");
		File launchDirectory = new File(newDirectory, "launch");
		File paramDirectory = new File(newDirectory, "param");
		return scriptsDirectory.mkdirs() && launchDirectory.mkdir() && paramDirectory.mkdir();
	}

	/**
	 * Create a default package.xml file
	 */
	void createPackageXML() {
		LOGGER.info("Creating package.xml");
		Template template = ve.getTemplate(PACKAGE_XML_TEMPLATE_FILE);
		VelocityContext context = new VelocityContext();
		context.put("packageName", ROS_PKG_DEAFULT_NAME);
		BufferedWriter writer;
		try {
			writer =  Files.newBufferedWriter(Paths.get(outputDir + rosPkgName + "/package.xml"));
			template.merge(context, writer);
			writer.flush();
			writer.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
	
	/**
	 * Create a default CMakeLists.txt file
	 */
	void createCMakeLists() {
		LOGGER.info("Creating CMakeLists.txt");
		Template template = ve.getTemplate(CMAKELISTS_TEMPLATE_FILE);
		VelocityContext context = new VelocityContext();
		context.put("packageName", ROS_PKG_DEAFULT_NAME);
		BufferedWriter writer;
		try {
			writer = Files.newBufferedWriter(Paths.get(outputDir + rosPkgName + "/CMakeLists.txt"));
			template.merge(context, writer);
			writer.flush();
			writer.close();
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

	/**
	 * Generate the SMACH python code from scxml file description of a FSM
	 */
	public boolean generate() {
		createROSPackage();
		LOGGER.info("ROS package created");

		createPackageXML();
		LOGGER.info("package.xml created");
		
		createCMakeLists();
		LOGGER.info("CMakeLists.txt created");

		// Create a list of custom actions, add as many as are needed
		List<CustomAction> customActions = new ArrayList<CustomAction>();
		// CustomAction ca = new
		// CustomAction("http://my.custom-actions.domain/cpswarm/CUSTOM", "input",
		// Input.class);
		// customActions.add(ca);
		try {
			SCXML scxml = null;
			try {
				LOGGER.info("Loading state machine...");
				LOGGER.info("path: " + inputPath);
				InputStream in = Files.newInputStream(Paths.get(inputPath));
				scxml = SCXMLReader.read(in, new SCXMLReader.Configuration(null, null, customActions));
			} catch (ModelException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			} catch (XMLStreamException e1) {
				// TODO Auto-generated catch block
				e1.printStackTrace();
			}

			/*
			 * Make a Context object and populate it.
			 */
			VelocityContext context = new VelocityContext();
			context.put("scxml", scxml);

			// List<State> innerFSM = new LinkedList<>();
			// context.put("list", innerFSM);

			/*
			 * make a writer, and merge the template 'against' the context
			 */
			BufferedWriter writer = Files.newBufferedWriter(Paths.get(outputDir + rosPkgName + "/scripts/" + SMACH_FILE_NAME));
			template.merge(context, writer);
			LOGGER.info("Writing code in: " + outputDir + rosPkgName + "/scripts/" + SMACH_FILE_NAME);
			writer.flush();
			writer.close();
		} catch (IOException e) {
			e.printStackTrace();
			return false;
		}
		return true;
	}
}
