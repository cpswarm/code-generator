package it.links.pert.codegen.model.ros;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.File;
import java.io.IOException;

import org.junit.jupiter.api.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;

import it.links.pert.codegen.json.RosADFReader;

class RosADFTest {

	private static final RosADFReader reader = new RosADFReader();

	private static final Logger LOGGER = LoggerFactory.getLogger(RosADFTest.class.getName());

	private RosADF parse(final String adfName) throws JsonParseException, JsonMappingException, IOException {
		final File adfile = new File(RosADFTest.class.getResource(adfName).getFile());
		return reader.read(adfile);
	}

	@Test
	final void testGetFunctionByName() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testGetFunctionByName test----------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		try {
			final RosADF adf = parse("uav_ADF_test1.json");
			assertNotNull(adf.getFunctionByName("uav_mavros_takeoff"));
			assertNotNull(adf.getFunctionByName("uav_mavros_land"));
			assertNotNull(adf.getFunctionByName("auction_action"));
		} catch (IOException e) {
			fail("Error " +e);
		}	
	}
}
