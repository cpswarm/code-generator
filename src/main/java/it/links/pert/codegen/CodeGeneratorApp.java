package it.links.pert.codegen;

import org.slf4j.LoggerFactory;
import org.slf4j.Logger;
import it.links.pert.codegen.generator.CodeGenerator;
import it.links.pert.codegen.scxml.SCXML2RosGenerator;

public class CodeGeneratorApp {

	private static final Logger LOGGER = LoggerFactory.getLogger(CodeGeneratorApp.class.getName());

	public static void main(String[] args) {

		String inputPath = null;
		String outputDir = null;
		String runtimeEnv = null;

		int i = 0;
		String arg;

		while (i < args.length && args[i].startsWith("--")) {
			arg = args[i++];
			if (arg.equals("--src")) {
				if (i < args.length) {
					inputPath = args[i++];
				} else {
					LOGGER.error("--src requires a file path");
				}
			} else if (arg.equals("--target")) {
				if (i < args.length) {
					outputDir = args[i++];
				} else {
					LOGGER.error("--target requires a directory path");
				}
			} else if (arg.equals("--env")) {
				if (i < args.length) {
					runtimeEnv = args[i++];
				} else {
					LOGGER.error("--env requires a valid Runtime Environment (e.g. ROS)");
				}
			}
		}

		CodeGenerator generator = null;

		switch (runtimeEnv) {
		case "ROS":
			generator = new SCXML2RosGenerator(inputPath, outputDir);
			break;
		default:
			generator = new SCXML2RosGenerator(inputPath, outputDir);
		}

		// URL url = App.class.getClassLoader().getResource(stateMachineFile);
		if (generator.generate())
			LOGGER.info("Code generated with success!!");
		else
			LOGGER.info("An error occured during code generation");
	}

}
