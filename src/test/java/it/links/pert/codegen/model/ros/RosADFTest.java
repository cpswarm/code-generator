package it.links.pert.codegen.model.ros;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.links.pert.codegen.json.RosADFReader;


class RosADFTest {
	
	private static final String INPUT_PATH = "/it/links/pert/codegen/json/data/uav_ADF_test1.json";
	private final static Path resourceDirectory = Paths.get("src", "test", "resources");
	private static RosADF adf;

	private static final Logger LOGGER = LoggerFactory.getLogger(RosADFTest.class.getName());
	
	@BeforeAll
	static void setUp() {
		final RosADFReader reader = new RosADFReader();
		File adfile = new File(resourceDirectory + INPUT_PATH);
		try {
			adf = reader.read(adfile);
		} catch (IOException e) {
			fail("Error:", e);
		}
	}

	@Test
	final void testGetFunctionByName() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testGetFunctionByName test--------------------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		assertNotNull(adf.getFunctionByName("uav_mavros_takeoff"));
		assertNotNull(adf.getFunctionByName("uav_mavros_land"));
		assertNotNull(adf.getFunctionByName("auction_action"));
	}

}
