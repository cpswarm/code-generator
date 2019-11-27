package it.links.pert.codegen.scxml;

import it.links.pert.codegen.generator.CodeGenerator;

public abstract class SCXML2RosGenerationMode implements CodeGenerator {

	protected SCXML2RosGenerator generator;

	public SCXML2RosGenerationMode(SCXML2RosGenerator generator) {
		this.generator = generator;
	}

	@Override
	public boolean generate() {
		return generator.generate();
	}

}
