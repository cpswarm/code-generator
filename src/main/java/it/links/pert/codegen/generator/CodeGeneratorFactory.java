package it.links.pert.codegen.generator;

import java.util.Map;

import it.links.pert.codegen.scxml.SCXML2RosGenerator;
import it.links.pert.codegen.scxml.SCXML2RosSimulationMode;

public class CodeGeneratorFactory {

	/**
	 * Factory to instantiate {@link CodeGenerator}
	 * 
	 * @param type      CodeGenerator type (e.g CodeGeneratorType.SCXML2ROS)
	 * @param outputDir Output directory path
	 * @param options   additional options dependent on {@link CodeGeneratorType}
	 * 
	 * @return a {@link CodeGenerator} instance
	 */
	public static CodeGenerator getInstance(CodeGeneratorType type, String outputDir, Map<String, String> options) {
		CodeGenerator generator = null;

		switch (type) {
		case SCXML2ROS:
			if (options.containsKey("rosPkgName")) {
				generator = new SCXML2RosGenerator(options.get("scxmlPath"), options.get("adfPath"), outputDir,
						options.get("rosPkgName"));
			} else {
				generator = new SCXML2RosGenerator(options.get("scxmlPath"), options.get("adfPath"), outputDir);
			}
			if (options.containsKey("mode")) {
				String mode = options.get("mode");
				if (mode != null && "simulation".contentEquals(mode)) {
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
