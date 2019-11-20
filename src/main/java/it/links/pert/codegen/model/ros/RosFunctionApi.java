
package it.links.pert.codegen.model.ros;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;

@JsonInclude(JsonInclude.Include.NON_NULL)
@JsonPropertyOrder({ "inputs", "outputs", "comm_model" })
public class RosFunctionApi {

	@JsonProperty("inputs")
	private List<Input> inputs = null;
	@JsonProperty("outputs")
	private List<Output> outputs = null;
	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("comm_model")
	private CommModel commModel;

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

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("comm_model")
	public CommModel getCommModel() {
		return commModel;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("comm_model")
	public void setCommModel(CommModel commModel) {
		this.commModel = commModel;
	}
}