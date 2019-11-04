/**
 * 
 */
package it.links.pert.codegen.json;

import java.io.IOException;
import java.io.Reader;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

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
public class ADFReader {
	
	/**
	 * 
	 * @param reader
	 * @return
	 * @throws IOException
	 * @throws ParseException
	 */
	public static JSONArray read(Reader reader) throws IOException, ParseException {
		JSONArray functionList = readInternal(reader);
		return functionList;
	}
	
	/**
	 * 
	 * @param reader
	 * @return
	 * @throws IOException
	 * @throws ParseException
	 */
	private static JSONArray readInternal(Reader reader) throws IOException, ParseException {
		JSONParser parser = new JSONParser();
		JSONObject jsonObject = (JSONObject) parser.parse(reader);
		JSONArray functionList = (JSONArray) jsonObject.get("functions");
		return functionList;
	}
}
