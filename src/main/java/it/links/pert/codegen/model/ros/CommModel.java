
package it.links.pert.codegen.model.ros;

import java.util.HashMap;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonValue;
import com.fasterxml.jackson.databind.annotation.JsonDeserialize;

@JsonDeserialize(using = CommModelDeserializer.class)
public class CommModel {

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("paradigm")
	private CommModel.Paradigm paradigm;
	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("definition")
	private Definition definition;


	public CommModel(Paradigm paradigm, Definition definition) {
		this.paradigm = paradigm;
		this.definition = definition;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("paradigm")
	public CommModel.Paradigm getParadigm() {
		return paradigm;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("paradigm")
	public void setParadigm(CommModel.Paradigm paradigm) {
		this.paradigm = paradigm;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("definition")
	public Definition getDefinition() {
		return definition;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("definition")
	public void setDefinition(Definition definition) {
		this.definition = definition;
	}

	public enum Paradigm {

		ROSACTION("rosaction"), ROSSERVICE("rosservice");
		private final String value;
		private final static Map<String, CommModel.Paradigm> CONSTANTS = new HashMap<String, CommModel.Paradigm>();

		static {
			for (CommModel.Paradigm c : values()) {
				CONSTANTS.put(c.value, c);
			}
		}

		private Paradigm(String value) {
			this.value = value;
		}

		@Override
		public String toString() {
			return this.value;
		}

		@JsonValue
		public String value() {
			return this.value;
		}

		@JsonCreator
		public static CommModel.Paradigm fromValue(String value) {
			CommModel.Paradigm constant = CONSTANTS.get(value);
			if (constant == null) {
				throw new IllegalArgumentException(value);
			} else {
				return constant;
			}
		}

	}

}