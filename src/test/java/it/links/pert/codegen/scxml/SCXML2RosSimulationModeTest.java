package it.links.pert.codegen.scxml;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.io.IOException;

import org.apache.commons.io.FileUtils;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

class SCXML2RosSimulationModeTest {

	private static final String OUTPUT_DIR = "test_tmp/";
	private static File testDirectory;
	private static boolean DEBUG = false;

	private static final Logger LOGGER = LoggerFactory.getLogger(SCXML2RosSimulationModeTest.class.getName());

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

	private SCXML2RosSimulationMode getGenerator(final String scxmlName, final String adfName, final String pkgName) {
		SCXML2RosGenerator generator = new SCXML2RosGenerator.SCXML2RosGeneratorBuilder(
				SCXML2RosSimulationModeTest.class.getResource(scxmlName).getFile(), OUTPUT_DIR)
						.initialRosPkgName(pkgName)
						.adfPath(SCXML2RosSimulationModeTest.class.getResource(adfName).getFile()).build();
		return new SCXML2RosSimulationMode(generator);
	}

	@Test
	public void testCreateROSPackage() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testCreateROSPackage test-----------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final SCXML2RosSimulationMode generator = getGenerator("data/UAV_sar_FSM2.xml", "data/uav_ADF_test.json",
				"createROSpkg");
		assertTrue(generator.createNewROSPackage());
	}

	@Test
	public void testGenerate() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testGenerate test-------------------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		final SCXML2RosSimulationMode generator = getGenerator("data/UAV_sar_FSM2.xml", "data/uav_ADF_test.json",
				"test_generate");
		assertTrue(generator.generate());
	}

}
