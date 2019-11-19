package it.links.pert.codegen.scxml;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

public class SCXMLExtractor {

	private static String STATE_TAG = "state";
	private static String DATA_TAG = "data";
	private static String FUNCTION_NAME_TAG = "name";

	public static NodeList getElementsByTagName(InputStream scxmlStream, String tagName)
			throws SAXException, IOException, ParserConfigurationException {
		return getElementsByTagNameInternal(scxmlStream, tagName);
	}

	/**
	 * Extract names of functions to be generated from a CPSwarm SCXML file
	 * 
	 * @param scxmlStream The {@link InputStream} supplying the SCXML document to
	 *                    parse.
	 * @return List names of function to be generated
	 * @throws SAXException
	 * @throws IOException
	 * @throws ParserConfigurationException
	 */
	public static List<String> getFunctionsToBeGenerated(InputStream scxmlStream)
			throws SAXException, IOException, ParserConfigurationException {

		List<String> functionNames = new ArrayList<String>();
		// Get all states
		NodeList stateList = getElementsByTagNameInternal(scxmlStream, STATE_TAG);
		for (int i = 0; i < stateList.getLength(); i++) {
			Element state = (Element) stateList.item(i);
			// Get data associated with this node
			NodeList dataList = state.getElementsByTagName(DATA_TAG);
			for (int j = 0; j < dataList.getLength(); j++) {
				Element data = (Element) dataList.item(j);
				String value = data.getAttribute("id");
				// Check if this data containf "adf" id
				if (value != null && "adf".contentEquals(value)) {
					// Extract function name from "data" tag
					Element nameTag = (Element) data.getElementsByTagName(FUNCTION_NAME_TAG).item(0);
					String name = nameTag.getFirstChild().getNodeValue();
					if (!functionNames.contains(name)) {
						functionNames.add(name);
					}
				}
			}
		}
		return functionNames;
	}

	private static NodeList getElementsByTagNameInternal(InputStream scxmlStream, String tagName)
			throws SAXException, IOException, ParserConfigurationException {
		Document scxmlDoc = DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(scxmlStream);
		return scxmlDoc.getElementsByTagName(tagName);
	}
}
