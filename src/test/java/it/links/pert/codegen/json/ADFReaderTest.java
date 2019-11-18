package it.links.pert.codegen.json;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.junit.jupiter.api.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;

class ADFReaderTest {

	private static final String FUNCTION_NAME = "name";
	private static final String INPUT_PATH = "/it/links/pert/codegen/json/data/uav_ADF_test1.json";
	private final Path resourceDirectory = Paths.get("src", "test", "resources");

	private static final Logger LOGGER = LoggerFactory.getLogger(ADFReader.class.getName());

	@Test
	public void testADFRead() throws Exception {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testADFRead test--------------------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");

		final Reader ADFileReader = Files.newBufferedReader(Paths.get(resourceDirectory + INPUT_PATH));
		JsonArray functionList = ADFReader.read(ADFileReader);
		assertEquals(3, functionList.size());
		JsonObject function0 = (JsonObject) functionList.get(0);
		assertEquals("uav_mavros_takeoff", function0.get(FUNCTION_NAME).getAsString());
		JsonObject function1 = (JsonObject) functionList.get(1);
		assertEquals("uav_mavros_land", function1.get(FUNCTION_NAME).getAsString());
		JsonObject function2 = (JsonObject) functionList.get(2);
		assertEquals("uav_assign_task", function2.get(FUNCTION_NAME).getAsString());
		JsonObject fnc_api = function2.get("api").getAsJsonObject();
		assertEquals("rosaction", fnc_api.get("comm_paradigm").getAsString());
	}

}
