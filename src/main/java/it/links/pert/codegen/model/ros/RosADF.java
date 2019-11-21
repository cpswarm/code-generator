package it.links.pert.codegen.model.ros;

import java.util.Optional;

import it.links.pert.codegen.model.ADF;

public class RosADF extends ADF<RosFunction> {

	public RosFunction getFunctionByName(String functionName) {
		Optional<RosFunction> optFunction = getFunctions().stream()
				.filter(function -> functionName.contentEquals(function.getName())).findFirst();
		return optFunction.orElse(null);
	}

}
