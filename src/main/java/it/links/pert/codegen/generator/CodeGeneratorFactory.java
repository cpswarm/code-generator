package it.links.pert.codegen.generator;

import java.util.Map;

import it.links.pert.codegen.scxml.SCXML2RosGenerator;

public class CodeGeneratorFactory {

	public static CodeGenerator getInstance(CodeGeneratorType type, String outputDir,
			Map<String, String> options) {
		CodeGenerator generator = null;

		switch (type) {
		case SCXML2ROS:
			if (options.containsKey("rosPkgName")) {
				generator = new SCXML2RosGenerator(options.get("scxmlPath"), options.get("adfPath"), outputDir,
						options.get("rosPkgName"));
			} else {
				generator = new SCXML2RosGenerator(options.get("scxmlPath"), options.get("adfPath"), outputDir);
			}
			break;

		default:
			// TODO throw dedicated exception
			break;
		}

		return generator;
	}
}
