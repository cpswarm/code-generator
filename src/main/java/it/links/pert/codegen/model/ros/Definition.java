package it.links.pert.codegen.model.ros;

import com.fasterxml.jackson.annotation.JsonProperty;


public abstract class Definition {
	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("name")
	private String name;
	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("class")
	private String _class;

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("name")
	public String getName() {
		return name;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("name")
	public void setName(String name) {
		this.name = name;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("class")
	public String getClass_() {
		return _class;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("class")
	public void setClass_(String _class) {
		this._class = _class;
	}
}
