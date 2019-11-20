package it.links.pert.codegen.model.ros;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.fasterxml.jackson.annotation.JsonCreator;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonValue;

import it.links.pert.codegen.model.Function;

public class RosFunction implements Function {

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
	@JsonProperty("description")
	private String description;
	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("category")
	private RosFunction.Category category;
	@JsonProperty("param_list")
	private List<Constant> paramList = null;
	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("api")
	private RosFunctionApi api;

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
	@JsonProperty("description")
	public String getDescription() {
		return description;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("description")
	public void setDescription(String description) {
		this.description = description;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("category")
	public String getCategory() {
		return category.value();
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("category")
	public void setCategory(RosFunction.Category category) {
		this.category = category;
	}

	@JsonProperty("param_list")
	public List<Constant> getParamList() {
		return paramList;
	}

	@JsonProperty("param_list")
	public void setParamList(List<Constant> paramList) {
		this.paramList = paramList;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("api")
	public RosFunctionApi getApi() {
		return api;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("api")
	public void setApi(RosFunctionApi api) {
		this.api = api;
	}

	public enum Category {

		ABSTRACTION_LIB("abstraction-lib"), SWARM_LIB("swarm-lib");
		private final String value;
		private final static Map<String, RosFunction.Category> CONSTANTS = new HashMap<String, RosFunction.Category>();

		static {
			for (RosFunction.Category c : values()) {
				CONSTANTS.put(c.value, c);
			}
		}

		private Category(String value) {
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
		public static RosFunction.Category fromValue(String value) {
			RosFunction.Category constant = CONSTANTS.get(value);
			if (constant == null) {
				throw new IllegalArgumentException(value);
			} else {
				return constant;
			}
		}

	}

}
