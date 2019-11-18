package it.links.pert.codegen.scxml;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.IOException;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;

import javax.xml.parsers.ParserConfigurationException;

import org.junit.jupiter.api.Test;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

class SCXMLExtractorTest {

	private static final String INPUT_PATH = "/it/links/pert/codegen/scxml/data/UAV_sar_FSM4.xml";
	private static final String TAG_NAME = "state";
	private final Path resourceDirectory = Paths.get("src", "test", "resources");

	private static final Logger LOGGER = LoggerFactory.getLogger(SCXMLExtractorTest.class.getName());

	@Test
	final void testGetElementsByTagName() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testGetElementsByTagName test-------------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		try (InputStream scxmlStream = Files.newInputStream(Paths.get(resourceDirectory + INPUT_PATH))) {
			NodeList stateList = SCXMLExtractor.getElementsByTagName(scxmlStream, TAG_NAME);
			assertEquals(2, stateList.getLength());
			Element state0 = (Element) stateList.item(0);
			assertEquals("TakeOff", state0.getAttribute("id"));
			Element state1 = (Element) stateList.item(1);
			assertEquals("Coverage", state1.getAttribute("id"));
		} catch (IOException | SAXException | ParserConfigurationException e) {
			fail("Error:", e);
		}
	}
	
	@Test
	final void testGetFunctionsToBeGenerated() {
		LOGGER.info("-----------------------------------------------------------------------------------------");
		LOGGER.info("--------------------Starting testGetFunctionsToBeGenerated test--------------------------");
		LOGGER.info("-----------------------------------------------------------------------------------------");
		try (InputStream scxmlStream = Files.newInputStream(Paths.get(resourceDirectory + INPUT_PATH))) {
			List<String> stateList = SCXMLExtractor.getFunctionsToBeGenerated(scxmlStream);
			assertEquals("uav_mavros_takeoff", stateList.get(0));
		} catch (IOException | SAXException | ParserConfigurationException e) {
			fail("Error:", e);
		}
	}

}
