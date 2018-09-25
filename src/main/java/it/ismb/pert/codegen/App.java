package it.ismb.pert.codegen;

import java.io.IOException;

import it.ismb.pert.codegen.scxml.SCXML2RosGenerator;

public class App {

	private final static String TEMPLATE_FILE = "template/state_machine.vm";
//	private final static String INPUT_DIR = "input/scxml/";
//	private final static String OUTPUT_DIR = "generated/";

	public static void main(String[] args) {

//		String inputFile = "/home/gprato/eclipse-workspace/cpswarm/code-generator/input/scxml/UAV_sar_FSM3.xml";
//		String outputFile = "/home/gprato/eclipse-workspace/cpswarm/code-generator/generated/uav_behavior_test.py";

		// String inputFile = "input/scxml/UGV_sar_FSM.xml";
		// String outputFile = "generated/ugv_behavior.py";

		// String inputFile = "input/scxml/logistic_FSM.xml";
		// String outputFile = "generated/logistic_behavior.py";
		
		String inputFile = null, outputFile = null;

		int i = 0;
		String arg;

		while (i < args.length && args[i].startsWith("-")) {
			arg = args[i++];
			if (arg.equals("-src")) {
				if (i < args.length) {
					// inputFile = INPUT_DIR + args[i++];
					inputFile = args[i++];
				} else {
					System.err.println("-src requires a file path");
				}
			} else if (arg.equals("-o")) {
				if (i < args.length) {
					// outputFile = OUTPUT_DIR + args[i++];
					outputFile = args[i++];
				} else {
					System.err.println("-o requires a file path");
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