package it.links.pert.codegen.model;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;

@JsonInclude(JsonInclude.Include.NON_NULL)
@JsonPropertyOrder({ "runtime-env", "functions" })
public abstract class ADF<E extends Function> {

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("runtime-env")
	private String runtimeEnv;
	@JsonProperty("functions")
	private List<E> functions;

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("runtime-env")
	public Object getRuntimeEnv() {
		return runtimeEnv;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("runtime-env")
	public void setRuntimeEnv(String runtimeEnv) {
		this.runtimeEnv = runtimeEnv;
	}

	@JsonProperty("functions")
	public List<E> getFunctions() {
		return functions;
	}

	@JsonProperty("functions")
	public void setFunctions(List<E> functions) {
		this.functions = functions;
	}
}
