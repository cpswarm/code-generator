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

@TestMethodOrder(OrderAnnotation.class)
class SCXML2RosTests {

	private static final String INPUT_PATH = "/it/links/pert/codegen/scxml/data/UAV_sar_FSM2.xml";
	private static final String OUTPUT_DIR = "test_tmp/";
	private static File testDirectory;
	private static SCXML2RosGenerator generator;

	@BeforeAll
	static void setUp() {
		//Set up temp directory to put outputs in
		testDirectory = new File(OUTPUT_DIR);
		if (!testDirectory.exists()) {
			testDirectory.mkdir();
		}
		//Initialize @SCXML2RosGenerator instance
		final Path resourceDirectory = Paths.get("src", "test", "resources");
		generator = new SCXML2RosGenerator(resourceDirectory + INPUT_PATH, OUTPUT_DIR);
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
		assertTrue(generator.createNewROSPackage());
	}

	@Test
	@Order(2)
	public void testCreateROSPackageWithExistingDir() {
		final File beahaviorPkg = new File(OUTPUT_DIR + SCXML2RosGenerator.ROS_PKG_DEAFULT_NAME);
		if (beahaviorPkg.exists())
			assertTrue(generator.createNewROSPackage());
		else {
			fail("Default package should already exist");
		}
	}

	@Test
	@Order(3)
	public void testGenerate() {
		assertTrue(generator.generate());
	}
}
