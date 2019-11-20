package it.links.pert.codegen.json;

import static org.junit.jupiter.api.Assertions.*;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.links.pert.codegen.model.ros.RosADF;

class RosADFReaderTest {

	private static final String INPUT_PATH = "/it/links/pert/codegen/json/data/uav_ADF_test1.json";
	private final Path resourceDirectory = Paths.get("src", "test", "resources");
	private static RosADFReader reader;

	private static final Logger LOGGER = LoggerFactory.getLogger(RosADFReaderTest.class.getName());

	@BeforeAll
	static void setUp() {
		reader = new RosADFReader();
	}

	@Test
	public void testADFRead() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testADFRead test--------------------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		File adfile = new File(resourceDirectory + INPUT_PATH);
		try {
			RosADF adf = reader.read(adfile);
			assertEquals(3, adf.getFunctions().size());
		} catch (IOException e) {
			fail("Error:", e);
		}
	}
}
