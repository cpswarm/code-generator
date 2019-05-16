package it.links.pert.codegen.scxml;

import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
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

import it.links.pert.codegen.generator.CodeGenerator;

public class SCXML2RosGenerator implements CodeGenerator {

	private final static String TEMPLATE_FILE_NAME = "template/state_machine.vm";
	private final static String SMACH_FILE_NAME = "behaviour.py";
	private final static String ROS_PKG_DEAFULT_NAME = "fsm_behaviour";

	private Template template;
	private String inputPath;
	private String outputDir;
	private String rosPkgName;

	public SCXML2RosGenerator(String inputPath, String outputDir) {
		this.inputPath = inputPath;
		this.outputDir = outputDir;
		rosPkgName = ROS_PKG_DEAFULT_NAME;
		VelocityEngine ve = new VelocityEngine();
		ve.setProperty(RuntimeConstants.RESOURCE_LOADER, "classpath");
		ve.setProperty("classpath.resource.loader.class", ClasspathResourceLoader.class.getName());
		template = ve.getTemplate(TEMPLATE_FILE_NAME);
		ve.setProperty("space.gobbling", "structured");
	}
	
	/**
	 * Create a new catkin package
	 * @return true if everything went well
	 */
	boolean createROSPackage() {
		File newDirectory = new File(outputDir + rosPkgName);
		int count = 0;
		String temp_dir = null;
		// If this package name already exist add a number at the end
		while (newDirectory.exists()) {
			count++;
			temp_dir = outputDir + rosPkgName + count;
			newDirectory = new File(temp_dir);
		}
		// Updated ros package directory name with his final name
		rosPkgName = temp_dir;
		File scriptsDirectory = new File(newDirectory, "scripts");
		File launchDirectory = new File(newDirectory, "launch");
		File paramDirectory = new File(newDirectory, "param");
		return scriptsDirectory.mkdirs() && launchDirectory.mkdir() && paramDirectory.mkdir();
	}

	/**
	 * Generate the SMACH python code from scxml file description of a FSM
	 */
	public void generate() throws IOException {
		createROSPackage();
		System.out.println("Created ROS package");

		// (1) Create a list of custom actions, add as many as are needed
		List<CustomAction> customActions = new ArrayList<CustomAction>();
		// CustomAction ca = new
		// CustomAction("http://my.custom-actions.domain/cpswarm/CUSTOM", "input",
		// Input.class);
		// customActions.add(ca);

		/*
		 * build a Document from our xml
		 */
		SCXML scxml = null;
		try {
			System.out.println("Loading state machine...");
			System.out.println("path: " + inputPath);
			InputStream in = new FileInputStream(inputPath);
			scxml = SCXMLReader.read(in, new SCXMLReader.Configuration(null, null, customActions));
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (ModelException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (XMLStreamException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		/*
		 * now, make a Context object and populate it.
		 */
		VelocityContext context = new VelocityContext();
		context.put("scxml", scxml);

		// List<State> innerFSM = new LinkedList<>();
		// context.put("list", innerFSM);

		/*
		 * make a writer, and merge the template 'against' the context
		 */
		// Write to console
		// Writer writer = null;
		// writer = new BufferedWriter(new OutputStreamWriter(System.out));

		FileWriter writer = new FileWriter(rosPkgName + "/scripts/" + SMACH_FILE_NAME);

		template.merge(context, writer);

		System.out.println("Writing code in: " + rosPkgName + "/scripts/" + SMACH_FILE_NAME);
		if (writer != null) {
			try {
				writer.flush();
				writer.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}
