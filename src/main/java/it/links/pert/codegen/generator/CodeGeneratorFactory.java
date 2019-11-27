package it.links.pert.codegen.generator;

import java.util.Map;

import it.links.pert.codegen.scxml.SCXML2RosGenerator;
import it.links.pert.codegen.scxml.SCXML2RosSimulationMode;

public class CodeGeneratorFactory {
	
	/**
	 * 
	 * @param type
	 * @param outputDir
	 * @param options
	 * @return
	 */
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
			if(options.containsKey("mode")) {
				if("simulation".contentEquals(options.get("mode"))) {
					generator = new SCXML2RosSimulationMode((SCXML2RosGenerator) generator);
				}
			}
			break;

		default:
			// TODO throw dedicated exception
			break;
		}

		return generator;
	}
}
