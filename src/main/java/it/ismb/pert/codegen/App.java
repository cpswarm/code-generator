package it.ismb.pert.codegen;

import java.io.IOException;

import it.ismb.pert.codegen.scxml.SCXML2RosGenerator;

public class App {

	private final static String TEMPLATE_FILE = "template/state_machine.vm";

	// Debug variables
	private final static boolean DEBUG = true;
	private final static String DEBUG_INPUT_DIR = "input/scxml/";
	private final static String DEBUG_OUTPUT_DIR = "generated/";

	public static void main(String[] args) {

		// String inputFile = "input/scxml/UGV_sar_FSM.xml";
		// String outputFile = "generated/ugv_behavior.py";

		// String inputFile = "input/scxml/logistic_FSM.xml";
		// String outputFile = "generated/logistic_behavior.py";

		String inputFile = null, outputFile = null;

		if (DEBUG) {
			inputFile = DEBUG_INPUT_DIR + "";
			outputFile = DEBUG_OUTPUT_DIR + "";
		} else {
			int i = 0;
			String arg;

			while (i < args.length && args[i].startsWith("--")) {
				arg = args[i++];
				if (arg.equals("--src")) {
					if (i < args.length) {
						inputFile = args[i++];
					} else {
						System.err.println("-src requires a file path");
					}
				} else if (arg.equals("--target")) {
					if (i < args.length) {
						outputFile = args[i++] + "/behavior.py";
					} else {
						System.err.println("--target requires a directory path");
					}
				}
			}
		}

		SCXML2RosGenerator generator = new SCXML2RosGenerator(TEMPLATE_FILE);
		// URL url = App.class.getClassLoader().getResource(stateMachineFile);
		try {
			generator.generate(inputFile, outputFile);
			System.out.println("Code generated with success!!");
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}

}
