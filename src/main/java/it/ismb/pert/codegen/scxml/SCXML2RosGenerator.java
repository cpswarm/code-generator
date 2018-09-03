package it.ismb.pert.codegen.scxml;

import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

import javax.xml.stream.XMLStreamException;

import org.apache.commons.scxml2.io.SCXMLReader;
import org.apache.commons.scxml2.model.CustomAction;
import org.apache.commons.scxml2.model.ModelException;
import org.apache.commons.scxml2.model.SCXML;
import org.apache.velocity.Template;
import org.apache.velocity.VelocityContext;
import org.apache.velocity.app.Velocity;

public class SCXML2RosGenerator {

	private Template template;

	public SCXML2RosGenerator(String templateFile) {
		template = Velocity.getTemplate(templateFile);
		Velocity.setProperty("space.gobbling", "structured");
	}

	public void generate(String inputPath, String dest) throws IOException {
		// (1) Create a list of custom actions, add as many as are needed
		List<CustomAction> customActions = new ArrayList<CustomAction>();
		//CustomAction ca = new CustomAction("http://my.custom-actions.domain/cpswarm/CUSTOM", "input", Input.class);
		//customActions.add(ca);

		/*
		 * build a Document from our xml
		 */

		SCXML scxml = null;
		try {
			System.out.println("Loading state machine...");
			System.out.println("path: " + inputPath);
			InputStream in = new FileInputStream(inputPath);
			scxml = SCXMLReader.read(in, new SCXMLReader.Configuration(null, null, customActions));
		} catch (IOException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (ModelException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		} catch (XMLStreamException e1) {
			// TODO Auto-generated catch block
			e1.printStackTrace();
		}

		/*
		 * now, make a Context object and populate it.
		 */

		VelocityContext context = new VelocityContext();
		context.put("scxml", scxml);
		
		//List<State> innerFSM = new LinkedList<>();
		//context.put("list", innerFSM);

		/*
		 * make a writer, and merge the template 'against' the context
		 */

		// Write to console
		// Writer writer = null;
		// writer = new BufferedWriter(new OutputStreamWriter(System.out));

		FileWriter writer = new FileWriter(dest);

		template.merge(context, writer);

		System.out.println("Writing code in: " + dest);
		if (writer != null) {
			try {
				writer.flush();
				writer.close();
			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
		}
	}
}
