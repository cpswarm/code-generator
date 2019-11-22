package it.links.pert.codegen.scxml;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.File;
import java.io.IOException;

import org.apache.commons.io.FileUtils;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TestMethodOrder(OrderAnnotation.class)
class SCXML2RosTest {

	private static final String OUTPUT_DIR = "test_tmp/";
	private static File testDirectory;
	private static boolean DEBUG = false;

	private static final Logger LOGGER = LoggerFactory.getLogger(SCXML2RosTest.class.getName());

	@BeforeAll
	static void setUp() {
		// Set up temp directory to put outputs in
		testDirectory = new File(OUTPUT_DIR);
		if (!testDirectory.exists()) {
			testDirectory.mkdir();
		}
	}

	@AfterAll
	static void tearDown() {
		if (!DEBUG && testDirectory.exists()) {
			try {
				FileUtils.deleteDirectory(testDirectory);
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
	}

	private SCXML2RosGenerator getGenerator(final String scxmlName, final String adfName, final String pkgName) {
		return new SCXML2RosGenerator(SCXML2RosTest.class.getResource(scxmlName).getFile(),
				SCXML2RosTest.class.getResource(adfName).getFile(), OUTPUT_DIR, pkgName);
	}

	private SCXML2RosGenerator getGenerator(final String scxmlName, final String adfName) {
		return new SCXML2RosGenerator(SCXML2RosTest.class.getResource(scxmlName).getFile(),
				SCXML2RosTest.class.getResource(adfName).getFile(), OUTPUT_DIR);
	}

	@Test
	public void testCreateROSPackage() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testCreateROSPackage test-----------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final SCXML2RosGenerator generator = getGenerator("data/UAV_sar_FSM2.xml", "data/uav_ADF_test.json",
				"createROSpkg");
		assertTrue(generator.createNewROSPackage());
	}

	@Test
	public void testCreateROSPackageWithExistingDir() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testCreateROSPackageWithExistingDir test--------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final SCXML2RosGenerator generator = getGenerator("data/UAV_sar_FSM2.xml", "data/uav_ADF_test.json",
				"existing_pkg");
		assertTrue(generator.createNewROSPackage());
		final File beahaviorPkg = new File(OUTPUT_DIR + generator.getLastGeneratedPkgName());
		if (beahaviorPkg.exists())
			assertTrue(generator.createNewROSPackage());
		else {
			fail("Default package should already exist");
		}
	}

	@Test
	public void testcreateROSFunctions1() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testcreateROSFunctions1 test---------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final SCXML2RosGenerator generator = getGenerator("data/UAV_sar_FSM4.xml", "data/uav_ADF_test.json",
				"rosFunction_takeoff");
		assertTrue(generator.createNewROSPackage());
		assertTrue(generator.createROSFunctions());
	}

	@Test
	public void testGenerate() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testGenerate test-------------------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final SCXML2RosGenerator generator = getGenerator("data/UAV_sar_FSM2.xml", "data/uav_ADF_test.json",
				"fsm_generate");
		assertTrue(generator.generate());
	}

	@Test
	public void testBehaviourGeneration1() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testBehaviourGeneration1 test-------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final SCXML2RosGenerator generator = getGenerator("data/action_goal_empty.xml", "data/uav_ADF_test.json",
				"fsm_gen");
		assertTrue(generator.generate());
		// Test generated behaviour against reference file
		final String base_dir_path = OUTPUT_DIR + generator.getLastGeneratedPkgName();
		final File behaviour = new File(base_dir_path + "/scripts/" + SCXML2RosGenerator.SMACH_FILE_NAME);
		final File behaviour_ref = new File(
				SCXML2RosTest.class.getResource("reference/ros/action_goal_empty.py").getFile());
		try {
			assertTrue(FileUtils.contentEquals(behaviour_ref, behaviour));
		} catch (IOException e) {
			fail("Behaviour files should be available");
		}
	}
	
	@Test
	public void testBehaviourGeneration2() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testBehaviourGeneration2 test-------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final SCXML2RosGenerator generator = getGenerator("data/service_goal_empty.xml", "data/uav_ADF_test.json",
				"fsm_gen");
		assertTrue(generator.generate());
		// Test generated behaviour against reference file
		final String base_dir_path = OUTPUT_DIR + generator.getLastGeneratedPkgName();
		final File behaviour = new File(base_dir_path + "/scripts/" + SCXML2RosGenerator.SMACH_FILE_NAME);
		final File behaviour_ref = new File(
				SCXML2RosTest.class.getResource("reference/ros/service_goal_empty.py").getFile());
		try {
			assertTrue(FileUtils.contentEquals(behaviour_ref, behaviour));
		} catch (IOException e) {
			fail("Behaviour files should be available");
		}
	}

	@Test
	public void validateCompleteGeneration() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting validateCompleteGeneration test---------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final SCXML2RosGenerator generator = getGenerator("data/UAV_sar_FSM2.xml", "data/uav_ADF_test.json");
		generator.generate();

		final String base_dir_path = OUTPUT_DIR + generator.getLastGeneratedPkgName();
		// Test generated behaviour against reference file
		final File behaviour = new File(base_dir_path + "/scripts/" + SCXML2RosGenerator.SMACH_FILE_NAME);
		final File behaviour_ref = new File(
				SCXML2RosTest.class.getResource("reference/ros/target_behaviour.py").getFile());
		try {
			assertTrue(FileUtils.contentEquals(behaviour_ref, behaviour));
		} catch (IOException e) {
			fail("Behaviour files should be available");
		}
		// Test generated CMakeLists.txt against reference file
		final File cmakeFile = new File(base_dir_path + "/CMakeLists.txt");
		final File cmakeFile_ref = new File(
				SCXML2RosTest.class.getResource("reference/ros/target_CMakeLists.txt").getFile());
		try {
			assertTrue(FileUtils.contentEquals(cmakeFile_ref, cmakeFile));
		} catch (IOException e) {
			fail("CMakeLists.txt files should be available");
		}
		// Test generated package.xml against reference file
		final File packageXMLFile = new File(base_dir_path + "/package.xml");
		final File packageXMLFile_ref = new File(
				SCXML2RosTest.class.getResource("reference/ros/target_package.xml").getFile());
		try {
			assertTrue(FileUtils.contentEquals(packageXMLFile_ref, packageXMLFile));
		} catch (IOException e) {
			fail("package.xml files should be available");
		}
		// Test generated package.xml against reference file
		final File rosFncFile = new File(base_dir_path + "/src/" + "uav_mavros_takeoff.cpp");
		final File rosFncFile_ref = new File(
				SCXML2RosTest.class.getResource("reference/ros/target_takeoff.cpp").getFile());
		try {
			assertTrue(FileUtils.contentEquals(rosFncFile_ref, rosFncFile));
		} catch (IOException e) {
			fail("package.xml files should be available");
		}
	}
}