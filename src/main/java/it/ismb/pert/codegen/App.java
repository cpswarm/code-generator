package it.ismb.pert.codegen;

import java.io.IOException;

import it.ismb.pert.codegen.scxml.SCXML2RosGenerator;

public class App {

	private final static String TEMPLATE_FILE = "template/state_machine.vm";
	private final static String INPUT_DIR = "input/scxml/";
	private final static String OUTPUT_DIR = "generated/";

	public static void main(String[] args) {
//		String outputFile = "generated/sar_behavior.py";
//		String inputFile = "input/scxml/sar_FSM.xml";

		 String inputFile  = "input/scxml/UAV_sar_FSM.xml";
		 String outputFile = "generated/uav_behavior.py";
		
		int i = 0;
		String arg;

		while (i < args.length && args[i].startsWith("-")) {
			arg = args[i++];
			if (arg.equals("-f")) {
				if (i < args.length)
					inputFile = INPUT_DIR + args[i++];
				else
					System.err.println("-f requires a filename");
			} else if (arg.equals("-o")) {
				if (i < args.length)
					outputFile = OUTPUT_DIR + args[i++];
				else
					System.err.println("-o requires a filename");
			}
		}

		SCXML2RosGenerator generator = new SCXML2RosGenerator(TEMPLATE_FILE);
		// URL url = App.class.getClassLoader().getResource(stateMachineFile);
		try {
			generator.generate(inputFile, outputFile);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		System.out.println("Code generated with success!!");
	}
	
//	private void init() {
//
//	}
 
}