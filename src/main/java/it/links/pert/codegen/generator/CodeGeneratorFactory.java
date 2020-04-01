package it.links.pert.codegen.generator;

import java.util.Map;

import it.links.pert.codegen.scxml.SCXML2RosGenerator;
import it.links.pert.codegen.scxml.SCXML2RosGenerator.SCXML2RosGeneratorBuilder;
import it.links.pert.codegen.scxml.SCXML2RosSimulationMode;

public final class CodeGeneratorFactory {

	private CodeGeneratorFactory() {
	}

	/**
	 * Factory to instantiate {@link CodeGenerator}
	 * 
	 * @param type      CodeGenerator type (e.g CodeGeneratorType.SCXML2ROS)
	 * @param outputDir Output directory path
	 * @param options   additional options dependent on {@link CodeGeneratorType}
	 * 
	 * @return a {@link CodeGenerator} instance
	 */
	public static CodeGenerator getInstance(final CodeGeneratorType type, final String outputDir,
			final Map<String, String> options) {
		CodeGenerator generator = null;

		switch (type) {
		case SCXML2ROS:
			final SCXML2RosGeneratorBuilder builder = new SCXML2RosGenerator.SCXML2RosGeneratorBuilder(
					options.get("scxmlPath"), outputDir);
			if (options.containsKey("rosPkgName")) {
				final String rosPkgName = options.get("rosPkgName");
				if (rosPkgName != null && !rosPkgName.isEmpty())
					builder.initialRosPkgName(rosPkgName);
			}
			if (options.containsKey("adfPath")) {
				final String adfPath = options.get("adfPath");
				if (adfPath != null && !adfPath.isEmpty())
					builder.withADFPath(adfPath);
			}
			generator = builder.build();
			if (options.containsKey("mode")) {
				final String mode = options.get("mode");
				String SIMULATION_MODE = "simulation";
				if (mode != null && SIMULATION_MODE.contentEquals(mode)) {
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
