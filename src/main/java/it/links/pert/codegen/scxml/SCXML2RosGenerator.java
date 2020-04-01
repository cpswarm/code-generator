package it.links.pert.codegen.scxml;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.ParserConfigurationException;
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
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.xml.sax.SAXException;

import it.links.pert.codegen.generator.CodeGenerator;
import it.links.pert.codegen.json.RosADFReader;
import it.links.pert.codegen.model.ros.RosADF;
import it.links.pert.codegen.model.ros.RosFunction;
import it.links.pert.codegen.scxml.exception.RosFunctionGenerationException;
import it.links.pert.codegen.scxml.helper.SCXMLExtractor;

public class SCXML2RosGenerator implements CodeGenerator {

	private static final Logger LOGGER = LoggerFactory.getLogger(SCXML2RosGenerator.class.getName());

	protected final static String FSM_TEMPLATE_FILE = "template/ros/state_machine.vm";
	protected final static String ACTION_SKELETON_TEMPLATE_FILE = "template/ros/skeleton/action_cpp.vm";
	protected final static String PACKAGE_XML_TEMPLATE_FILE = "template/ros/package_xml.vm";
	protected final static String CMAKELISTS_TEMPLATE_FILE = "template/ros/cmakelists.vm";
	protected final static String SMACH_FILE_NAME = "behaviour.py";
	protected final static String ROS_PKG_DEAFULT_NAME = "fsm_behaviour";

	// *** INTERNAL VARS ***
	/**
	 * Last generated package name (to allow multiple generation using the same //
	 * SCXML2RosGenerator instance)
	 */
	private String currentRosPkgName;
	private final VelocityEngine engine;

	private final String scxmlPath; // required
	private final String outputDir; // required
	private final String adfPath; // optional
	/**
	 * Initial name for new generated ROS packages
	 */
	private final String initialRosPkgName; // optional

	public static class SCXML2RosGeneratorBuilder {
		private final String scxmlPath;
		private final String outputDir;
		private String adfPath;
		private String initialRosPkgName;

		public SCXML2RosGeneratorBuilder(String scxmlPath, String outputDir) {
			this.scxmlPath = scxmlPath;
			this.outputDir = outputDir;
			this.initialRosPkgName = ROS_PKG_DEAFULT_NAME;
		}

		public SCXML2RosGeneratorBuilder adfPath(String adfPath) {
			this.adfPath = adfPath;
			return this;
		}

		public SCXML2RosGeneratorBuilder initialRosPkgName(String initialRosPkgName) {
			this.initialRosPkgName = initialRosPkgName;
			return this;
		}

		public SCXML2RosGenerator build() {
			return new SCXML2RosGenerator(this);
		}
	}

	public SCXML2RosGenerator(SCXML2RosGeneratorBuilder builder) {
		final File scxmlFile = new File(builder.scxmlPath);
		if (!scxmlFile.exists()) {
			throw new IllegalArgumentException("SCXML path must be an existing file");
		}
		this.scxmlPath = builder.scxmlPath;

		if (builder.outputDir.endsWith(File.separator)) {
			this.outputDir = builder.outputDir;
		} else {
			this.outputDir = builder.outputDir + File.separator;
		}

		this.initialRosPkgName = builder.initialRosPkgName;
		this.currentRosPkgName = builder.initialRosPkgName;

		if (builder.adfPath != null) {
			final File adfFile = new File(builder.adfPath);
			if (!adfFile.exists()) {
				throw new IllegalArgumentException("ADF path must be an existing file");
			}
		}
		this.adfPath = builder.adfPath;

		engine = new VelocityEngine();
		engine.setProperty(RuntimeConstants.RESOURCE_LOADERS, "classpath");
		engine.setProperty("resource.loader.classpath.class", ClasspathResourceLoader.class.getName());
		engine.setProperty("parser.space_gobbling", "lines");
	}

	protected String getOutputDir() {
		return outputDir;
	}

	/**
	 * 
	 * @return String name of the last generated ROS pkg
	 */
	protected String getLastGeneratedPkgName() {
		return currentRosPkgName;
	}

	/**
	 * Create a new minimal catkin ROS package
	 * 
	 * @return true if the package is correctly created
	 */
	protected boolean createNewROSPackage() {
		return makeROSPackageDirs() && createPackageXML() && createCMakeLists();
	}

	// ************************************************************************//

	/**
	 * Create a new ROS package directory structure
	 * 
	 * @return true if the package is correctly created
	 */
	private boolean makeROSPackageDirs() {
		LOGGER.info("Creating ROS package directories");
		File newDirectory = new File(outputDir + initialRosPkgName);
		int count = 1;
		String tmpDir = null;
		// If this package name already exists add a number at the end
		while (newDirectory.exists()) {
			currentRosPkgName = initialRosPkgName + count++;
			tmpDir = outputDir + currentRosPkgName;
			newDirectory = new File(tmpDir);
		}
		final File scriptsDirectory = new File(newDirectory, "scripts");
		final File srcDirectory = new File(newDirectory, "src");
		final File launchDirectory = new File(newDirectory, "launch");
		final File paramDirectory = new File(newDirectory, "param");
		return scriptsDirectory.mkdirs() && srcDirectory.mkdir() && launchDirectory.mkdir() && paramDirectory.mkdir()
				&& newDirectory.exists();
	}

	/**
	 * Create a default package.xml file
	 * 
	 * @return true if the file is correctly created
	 */
	private boolean createPackageXML() {
		LOGGER.info("Creating package.xml");
		final Template template = engine.getTemplate(PACKAGE_XML_TEMPLATE_FILE);
		final VelocityContext context = new VelocityContext();
		context.put("packageName", currentRosPkgName);
		final Path path = Paths.get(outputDir + currentRosPkgName + "/package.xml");
		try (BufferedWriter writer = Files.newBufferedWriter(path)) {
			template.merge(context, writer);
			writer.flush();
		} catch (IOException e) {
			LOGGER.error("Error:", e);
		}
		return Files.exists(path);
	}

	/**
	 * Create a default CMakeLists.txt file
	 * 
	 * @return true if the file is correctly created
	 */
	private boolean createCMakeLists() {
		LOGGER.info("Creating CMakeLists.txt");
		final Template template = engine.getTemplate(CMAKELISTS_TEMPLATE_FILE);
		final VelocityContext context = new VelocityContext();
		context.put("packageName", currentRosPkgName);
		final Path path = Paths.get(outputDir + currentRosPkgName + "/CMakeLists.txt");
		try (BufferedWriter writer = Files.newBufferedWriter(path)) {
			template.merge(context, writer);
			writer.flush();
		} catch (IOException e) {
			LOGGER.error("Error:", e);
		}
		return Files.exists(path);
	}

	// ************************************************************************//

	/**
	 * Generate ROS functions skeleton Functions to be generated are extracted from
	 * SCXML file Data to generate function code are extracted from ADF json file
	 * 
	 * @return boolean Return true if generation process completed correctly
	 */
	protected boolean createROSFunctions() {
		boolean success = false;

		try (InputStream scxmlStream = Files.newInputStream(Paths.get(scxmlPath))) {
			// Get function names from SCXML file
			final List<String> functionNames = SCXMLExtractor.getFunctionsToBeGenerated(scxmlStream);
			if (!functionNames.isEmpty()) {
				RosADFReader adfReader = new RosADFReader();
				final RosADF adf = adfReader.read(new File(adfPath));
				for (final String name : functionNames) {
					// Extract function description from ADF
					RosFunction fncDescription = adf.getFunctionByName(name);
					if (fncDescription != null) {
						// Generate function skeleton
						if (!createROSActionSkeleton(fncDescription)) {
							// If operation FAILS return immediately
							throw new RosFunctionGenerationException("ROS function generation error");
						}
					} else {
						throw new RosFunctionGenerationException(
								"Function description not available for \'" + name + "\'");
					}
				}
			}
			success = true;
		} catch (IOException | SAXException | ParserConfigurationException | RosFunctionGenerationException e) {
			LOGGER.error("Error:", e);
			success = false;
		}

		return success;
	}

	private boolean createROSActionSkeleton(final RosFunction fncDescription) {
		LOGGER.info("Generating ROS Action Skeleton...");
		boolean success = false;

		final VelocityContext context = new VelocityContext();
		context.put("fncDescription", fncDescription);
		/*
		 * make a writer, and merge the template 'against' the context
		 */
		final String path = outputDir + currentRosPkgName + "/src/" + fncDescription.getName() + ".cpp";
		try (BufferedWriter writer = Files.newBufferedWriter(Paths.get(path));) {
			final Template template = engine.getTemplate(ACTION_SKELETON_TEMPLATE_FILE);
			template.merge(context, writer);
			LOGGER.info("Writing ROS function in: " + path);
			writer.flush();
			success = true;
		} catch (IOException e) {
			LOGGER.error("Error:", e);
			success = false;
		}
		return success;
	}

	// ************************************************************************//
	/**
	 * Generate SMACH FSM (python) Data to generate the code are extracted from
	 * SCXML file
	 * 
	 * @return boolean Return true if generation process completed correctly
	 */
	protected boolean generateFSMBehavior() {
		LOGGER.info("Generating FSM behavior...");
		boolean success = false;

		// Create a list of custom actions, add as many as are needed
		final List<CustomAction> customActions = new ArrayList<>();
		// CustomAction ca = new
		// CustomAction("http://my.custom-actions.domain/cpswarm/CUSTOM", "input",
		// Input.class);
		// customActions.add(ca);
		try (InputStream inputFile = Files.newInputStream(Paths.get(scxmlPath))) {
			LOGGER.info("Loading state machine...");
			LOGGER.info("path: " + scxmlPath);
			SCXML scxml;
			scxml = SCXMLReader.read(inputFile, new SCXMLReader.Configuration(null, null, customActions));
			/*
			 * Make a Context object and populate it.
			 */
			final VelocityContext context = new VelocityContext();
			context.put("scxml", scxml);
			/*
			 * make a writer, and merge the template 'against' the context
			 */
			final String path = outputDir + currentRosPkgName + "/scripts/" + SMACH_FILE_NAME;
			try (BufferedWriter writer = Files.newBufferedWriter(Paths.get(path));) {
				final Template template = engine.getTemplate(FSM_TEMPLATE_FILE);
				template.merge(context, writer);
				LOGGER.info("Writing FSM code in: " + path);
				writer.flush();
				success = true;
			}
		} catch (IOException | ModelException | XMLStreamException e) {
			LOGGER.error("Error:", e);
			success = false;
		}
		return success;
	}

	/**
	 * Generate the SMACH python code from SCXML file description of a FSM and put
	 * it in a new catkin ROS package
	 * 
	 * @return true if the code is correctly generated
	 */
	@Override
	public boolean generate() {
		boolean success;
		success = createNewROSPackage() && createROSFunctions() && generateFSMBehavior();
		return success;
	}
}
