package it.links.pert.codegen.model.ros;

import java.util.List;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;

@JsonInclude(JsonInclude.Include.NON_NULL)
@JsonPropertyOrder({ "fields" })
public class Goal {
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
