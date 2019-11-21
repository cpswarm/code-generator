package it.links.pert.codegen;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.links.pert.codegen.generator.CodeGenerator;
import it.links.pert.codegen.scxml.SCXML2RosGenerator;

public final class CodeGeneratorLauncher {

	private static final Logger LOGGER = LoggerFactory.getLogger(CodeGeneratorLauncher.class.getName());

	/*
	 * Private constructor
	 */
	private CodeGeneratorLauncher() {
	}

	public static void main(String[] args) {

		String scxmlPath = null;
		String outputDir = null;
		String runtimeEnv = null;
		String adfPath = null;

		int i = 0;
		String arg;

		while (i < args.length && args[i].startsWith("--")) {
			arg = args[i++];
			if ("--scxml".equals(arg)) {
				if (i < args.length) {
					scxmlPath = args[i++];
				} else {
					LOGGER.error("--scxml requires a valid file path");
				}
			} else if ("--adf".equals(arg)) {
				if (i < args.length) {
					adfPath = args[i++];
				} else {
					LOGGER.error("--adf requires a valid file path");
				}
			} else if ("--output".equals(arg)) {
				if (i < args.length) {
					outputDir = args[i++];
				} else {
					LOGGER.error("--output requires an existing directory path");
				}
			} else if ("--env".equals(arg)) {
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
			generator = new SCXML2RosGenerator(scxmlPath, adfPath, outputDir);
			break;
		default:
			generator = new SCXML2RosGenerator(scxmlPath, adfPath, outputDir);
		}

		// URL url = App.class.getClassLoader().getResource(stateMachineFile);
		if (generator.generate()) {
			LOGGER.info("Code generated with success!!");
		} else {
			LOGGER.info("An error occured during code generation");
		}
	}

}
