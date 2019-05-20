package it.links.pert.codegen;

import it.links.pert.codegen.generator.CodeGenerator;
import it.links.pert.codegen.scxml.SCXML2RosGenerator;

public class App {

	public static void main(String[] args) {

		String inputPath = null, outputDir = null, runtimeEnv = null;

		int i = 0;
		String arg;

		while (i < args.length && args[i].startsWith("--")) {
			arg = args[i++];
			if (arg.equals("--src")) {
				if (i < args.length) {
					inputPath = args[i++];
				} else {
					System.err.println("--src requires a file path");
				}
			} else if (arg.equals("--target")) {
				if (i < args.length) {
					outputDir = args[i++];
				} else {
					System.err.println("--target requires a directory path");
				}
			} else if (arg.equals("--env")) {
				if (i < args.length) {
					runtimeEnv = args[i++];
				} else {
					System.err.println("--env requires a valid Runtime Environment (e.g. ROS)");
				}
			}
		}
		
		CodeGenerator generator = null;
		
		switch(runtimeEnv) {
			case "ROS":
				generator =  new SCXML2RosGenerator(inputPath, outputDir);
				break;
			default:
				generator =  new SCXML2RosGenerator(inputPath, outputDir);
		}

		// URL url = App.class.getClassLoader().getResource(stateMachineFile);
		if(generator.generate())
			System.out.println("Code generated with success!!");
		else
			System.out.println("An error occured during code generation");
	}

}
