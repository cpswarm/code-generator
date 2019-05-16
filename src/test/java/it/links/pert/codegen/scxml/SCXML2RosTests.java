package it.links.pert.codegen.scxml;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.File;
import java.io.IOException;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class SCXML2RosTests {

	private static final String INPUT_PATH = "input/scxml/test/UAV_sar_FSM.xml";
	private static final String OUTPUT_DIR = "generated/test/";
	private final SCXML2RosGenerator generator = new SCXML2RosGenerator(INPUT_PATH, OUTPUT_DIR);

	@BeforeAll
	static void createTestDir() {
		File testDirectory = new File(OUTPUT_DIR);
		if (!testDirectory.exists()) {
			testDirectory.mkdir();
		}
	}

	@Test
	public void testCreateROSPackage() {
		assertTrue(generator.createROSPackage());
	}

	@Test
	public void testGenerate() throws IOException {
		generator.generate();
	}

}
