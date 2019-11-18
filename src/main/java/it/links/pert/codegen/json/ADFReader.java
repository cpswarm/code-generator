/**
 * 
 */
package it.links.pert.codegen.json;

import java.io.Reader;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;



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
	 */
	public static JsonArray read(Reader reader) {
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
}
