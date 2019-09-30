package it.links.pert.codegen.scxml;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.MethodOrderer.OrderAnnotation;
import org.junit.jupiter.api.Order;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestMethodOrder;

@TestMethodOrder(OrderAnnotation.class)
class SCXML2RosTests {

	private static final String INPUT_PATH = "/it/links/pert/codegen/scxml/data/UAV_sar_FSM.xml";
	private static final String OUTPUT_DIR = "generated/test/";
	private static SCXML2RosGenerator generator;

	@BeforeAll
	static void createTestDir() {
		File testDirectory = new File(OUTPUT_DIR);
		if (!testDirectory.exists()) {
			testDirectory.mkdir();
		}
	}
	
	@BeforeAll
	static void createGeneratorInstance() {
		Path resourceDirectory = Paths.get("src","test","resources");
		generator = new SCXML2RosGenerator(resourceDirectory + INPUT_PATH, OUTPUT_DIR);
	}

	@Test
	@Order(1)
	public void testCreateROSPackage() {
		assertTrue(generator.createROSPackage());
	}
	
	@Test
	@Order(2)
	public void testCreateROSPackageWithExistingDir() {
		File beahaviour_pkg = new File(OUTPUT_DIR + SCXML2RosGenerator.ROS_PKG_DEAFULT_NAME);
		if(beahaviour_pkg.exists())
			assertTrue(generator.createROSPackage());
	}

	@Test
	@Order(3)
	public void testGenerate() {
		generator.generate();
	}
	
	

}
