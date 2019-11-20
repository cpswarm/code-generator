/**
 * 
 */
package it.links.pert.codegen.json;

import java.io.File;
import java.io.IOException;
import java.io.Reader;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import it.links.pert.codegen.model.ros.RosADF;
import it.links.pert.codegen.model.ros.RosFunction;



/**
 * The ADFReader provides the ability to read an Abstraction Description File document.
 * 
 * The ADFReader assumes that the ADF document to be
 * parsed is well-formed and correct. If that assumption does not hold,
 * any subsequent behavior is undefined.
 * 
 * @author gprato
 *
 */
public class RosADFReader implements ADFReader<RosFunction> {
	
	/**
	 * 
	 * @param reader
	 * @return
	 */
	public static JsonArray read2(Reader reader) {
		JsonArray functionList = readInternal(reader);
		return functionList;
	}
	
	/**
	 * 
	 * @param reader
	 * @return
	 */
	private static JsonArray readInternal(Reader reader) {
		JsonObject jsonObject = JsonParser.parseReader(reader).getAsJsonObject();
		JsonArray functionList = (JsonArray) jsonObject.get("functions");
		
		
		return functionList;
	}

	@Override
	public RosADF read(File jsonADF) throws JsonParseException, JsonMappingException, IOException {
		ObjectMapper mapper = new ObjectMapper();
		RosADF adf = mapper.readValue(jsonADF, RosADF.class);
		return adf;
	}

}
