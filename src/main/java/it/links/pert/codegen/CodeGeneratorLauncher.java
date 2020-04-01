package it.links.pert.codegen;

import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.Callable;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import it.links.pert.codegen.generator.CodeGenerator;
import it.links.pert.codegen.generator.CodeGeneratorFactory;
import it.links.pert.codegen.generator.CodeGeneratorType;
import picocli.CommandLine;
import picocli.CommandLine.Command;
import picocli.CommandLine.Option;

@Command(mixinStandardHelpOptions = true, sortOptions = false, version = "1.0")
public final class CodeGeneratorLauncher implements Callable<Boolean> {

	@Option(names = { "--env", "-e" }, description = "Target runtime environment (default: ${DEFAULT-VALUE})", defaultValue = "ROS")
	public String runtimeEnv;
	@Option(names = { "--output", "-o" }, required = true, description = "Output directory path")
	public String outputDir;
	@Option(names = { "--scxml" }, required = true, description = "SCXML behavior file path", paramLabel = "<filePath>")
	public String scxmlPath;
	@Option(names = { "--adf" }, description = "Abstraction Description File (ADF) path", paramLabel = "<filePath>")
	public String adfPath;

	private static final Logger LOGGER = LoggerFactory.getLogger(CodeGeneratorLauncher.class.getName());

	public static void main(String[] args) {
		System.exit(new CommandLine(new CodeGeneratorLauncher()).execute(args));
	}

	@Override
	public Boolean call() throws Exception {
		boolean success = false;

		final Map<String, String> options = new HashMap<String, String>();
		options.put("scxmlPath", scxmlPath);
		options.put("adfPath", adfPath);

		CodeGenerator generator = null;
		switch (runtimeEnv) {
		case "ROS":
			generator = CodeGeneratorFactory.getInstance(CodeGeneratorType.SCXML2ROS, outputDir, options);
			break;
		default:
			generator = CodeGeneratorFactory.getInstance(CodeGeneratorType.SCXML2ROS, outputDir, options);
		}

		success = generator.generate();
		if (success) {
			LOGGER.info("Code generated with SUCCESS!!");
		} else {
			LOGGER.info("An error occured during code generation");
		}
		return success;
	}

}
