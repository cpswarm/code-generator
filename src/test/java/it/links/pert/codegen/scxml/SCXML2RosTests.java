package it.links.pert.codegen.scxml;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.apache.commons.io.FileUtils;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TestMethodOrder(OrderAnnotation.class)
class SCXML2RosTests {

	private static final String OUTPUT_DIR = "test_tmp/";
	private static final String REF_FILE_DIR = "/it/links/pert/codegen/scxml/reference/ros/";
	private static final Path resourceDirectory = Paths.get("src", "test", "resources");
	private static File testDirectory;

	private static final Logger LOGGER = LoggerFactory.getLogger(SCXML2RosTests.class.getName());

	@BeforeAll
	static void setUp() {
		// Set up temp directory to put outputs in
		testDirectory = new File(OUTPUT_DIR);
		if (!testDirectory.exists()) {
			testDirectory.mkdir();
		}
		// Initialize @SCXML2RosGenerator instance
	}

	@AfterAll
	static void tearDown() {
		if (testDirectory.exists()) {
			try {
				FileUtils.deleteDirectory(testDirectory);
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}

	@Test
	@Order(1)
	public void testCreateROSPackage() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testCreateROSPackage test-----------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final String scxmlPath = "/it/links/pert/codegen/scxml/data/UAV_sar_FSM2.xml";
		final String adfPath = "/it/links/pert/codegen/scxml/data/uav_ADF_test.json";
		final SCXML2RosGenerator generator = new SCXML2RosGenerator(resourceDirectory + scxmlPath,
				resourceDirectory + adfPath, OUTPUT_DIR, "createROSpkg");
		assertTrue(generator.createNewROSPackage());
	}

	@Test
	@Order(2)
	public void testCreateROSPackageWithExistingDir() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testCreateROSPackageWithExistingDir test--------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final String scxmlPath = "/it/links/pert/codegen/scxml/data/UAV_sar_FSM2.xml";
		final String adfPath = "/it/links/pert/codegen/scxml/data/uav_ADF_test.json";
		final SCXML2RosGenerator generator = new SCXML2RosGenerator(resourceDirectory + scxmlPath,
				resourceDirectory + adfPath, OUTPUT_DIR, "existing_pkg");
		assertTrue(generator.createNewROSPackage());
		final File beahaviorPkg = new File(OUTPUT_DIR + generator.getLastGeneratedPkgName());
		if (beahaviorPkg.exists())
			assertTrue(generator.createNewROSPackage());
		else {
			fail("Default package should already exist");
		}
	}

	@Test
	@Order(3)
	public void testcreateROSFunctions1() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testcreateROSFunctions test---------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final String scxmlPath = "/it/links/pert/codegen/scxml/data/UAV_sar_FSM2.xml";
		final String adfPath = "/it/links/pert/codegen/scxml/data/uav_ADF_test.json";
		final SCXML2RosGenerator generator = new SCXML2RosGenerator(resourceDirectory + scxmlPath,
				resourceDirectory + adfPath, OUTPUT_DIR, "rosFunction1");
		assertTrue(generator.createNewROSPackage());
		assertTrue(generator.createROSFunctions());
	}

	@Test
	@Order(4)
	public void testGenerate() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testGenerate test-------------------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final String scxmlPath = "/it/links/pert/codegen/scxml/data/UAV_sar_FSM2.xml";
		final String adfPath = "/it/links/pert/codegen/scxml/data/uav_ADF_test.json";
		final SCXML2RosGenerator generator = new SCXML2RosGenerator(resourceDirectory + scxmlPath,
				resourceDirectory + adfPath, OUTPUT_DIR, "fsm_generate");
		assertTrue(generator.generate());
	}

	@Test
	@Order(5)
	public void validateGeneratedFiles() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting validateGeneratedFiles test---------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final String scxmlPath = "/it/links/pert/codegen/scxml/data/UAV_sar_FSM2.xml";
		final String adfPath = "/it/links/pert/codegen/scxml/data/uav_ADF_test.json";
		final SCXML2RosGenerator generator = new SCXML2RosGenerator(resourceDirectory + scxmlPath,
				resourceDirectory + adfPath, OUTPUT_DIR);
		generator.generate();

		final String base_dir_path = OUTPUT_DIR + generator.getLastGeneratedPkgName();
		final Path resourceDirectory = Paths.get("src", "test", "resources");
		// Test generated behaviour against reference file
		final File behaviour = new File(base_dir_path + "/scripts/" + SCXML2RosGenerator.SMACH_FILE_NAME);
		final File behaviour_ref = new File(resourceDirectory + REF_FILE_DIR + "target_behaviour.py");
		try {
			assertTrue(FileUtils.contentEquals(behaviour_ref, behaviour));
		} catch (IOException e) {
			fail("Behaviour files should be available");
		}
		// Test generated CMakeLists.txt against reference file
		final File cmakeFile = new File(base_dir_path + "/CMakeLists.txt");
		final File cmakeFile_ref = new File(resourceDirectory + REF_FILE_DIR + "target_CMakeLists.txt");
		try {
			assertTrue(FileUtils.contentEquals(cmakeFile_ref, cmakeFile));
		} catch (IOException e) {
			fail("CMakeLists.txt files should be available");
		}
		// Test generated package.xml against reference file
		final File packageXMLFile = new File(base_dir_path + "/package.xml");
		final File packageXMLFile_ref = new File(resourceDirectory + REF_FILE_DIR + "target_package.xml");
		try {
			assertTrue(FileUtils.contentEquals(packageXMLFile_ref, packageXMLFile));
		} catch (IOException e) {
			fail("package.xml files should be available");
		}
		// Test generated package.xml against reference file
		final File rosFncFile = new File(base_dir_path + "/src/" + "uav_mavros_takeoff.cpp");
		final File rosFncFile_ref = new File(resourceDirectory + REF_FILE_DIR + "target_takeoff.cpp");
		try {
			assertTrue(FileUtils.contentEquals(rosFncFile_ref, rosFncFile));
		} catch (IOException e) {
			fail("package.xml files should be available");
		}
	}
}
