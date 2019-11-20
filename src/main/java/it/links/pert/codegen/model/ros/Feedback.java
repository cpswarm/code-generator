package it.links.pert.codegen.model.ros;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonProperty;

public class Feedback {
	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("fields")
	private List<Field> fields = null;

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("fields")
	public List<Field> getFields() {
		return fields;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("fields")
	public void setFields(List<Field> fields) {
		this.fields = fields;
	}
}
