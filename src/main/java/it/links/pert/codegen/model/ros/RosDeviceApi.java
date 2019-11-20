package it.links.pert.codegen.model.ros;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;

@JsonInclude(JsonInclude.Include.NON_NULL)
@JsonPropertyOrder({ "inputs", "outputs", "services" })
public class RosDeviceApi {
	@JsonProperty("inputs")
	private List<Input> inputs = null;
	@JsonProperty("outputs")
	private List<Output> outputs = null;
	@JsonProperty("services")
	private List<Service> services = null;

	@JsonProperty("inputs")
	public List<Input> getInputs() {
		return inputs;
	}

	@JsonProperty("inputs")
	public void setInputs(List<Input> inputs) {
		this.inputs = inputs;
	}

	@JsonProperty("outputs")
	public List<Output> getOutputs() {
		return outputs;
	}

	@JsonProperty("outputs")
	public void setOutputs(List<Output> outputs) {
		this.outputs = outputs;
	}

	@JsonProperty("services")
	public List<Service> getServices() {
		return services;
	}

	@JsonProperty("services")
	public void setServices(List<Service> services) {
		this.services = services;
	}
}
