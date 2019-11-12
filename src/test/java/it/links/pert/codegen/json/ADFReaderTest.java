package it.links.pert.codegen.json;

import java.io.File;
import java.io.Reader;
import java.nio.file.Files;
import java.nio.file.Paths;

import org.apache.commons.io.FileUtils;
import org.json.simple.JSONArray;
import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ADFReaderTest {

	private static final String INPUT_PATH = "/it/links/pert/codegen/json/data/";
	private static final String OUTPUT_DIR = "test_tmp/";
	private static final String REF_FILE_DIR = "/it/links/pert/codegen/json/reference/ros/";
	private static File testDirectory;

	@BeforeAll
	static void setUpBeforeClass() throws Exception {
		// Set up temp directory to put outputs in
		testDirectory = new File(OUTPUT_DIR);
		if (!testDirectory.exists()) {
			testDirectory.mkdir();
		}
	}

	@AfterAll
	static void tearDownAfterClass() throws Exception {
		FileUtils.deleteDirectory(testDirectory);
	}

	@Test
	public void testRead() throws Exception {
		//TODO to be completed
		final Reader ADFileReader = Files.newBufferedReader(Paths.get(""));
		JSONArray functionList = ADFReader.read(ADFileReader);
	}

}
