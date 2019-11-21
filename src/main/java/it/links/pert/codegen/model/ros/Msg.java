
package it.links.pert.codegen.model.ros;

import java.util.List;
import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;

@JsonInclude(JsonInclude.Include.NON_NULL)
@JsonPropertyOrder({ "class", "constants", "fields" })
public class Msg {

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("class")
	private String _class;
	@JsonProperty("constants")
	private List<Constant> constants = null;
	@JsonProperty("fields")
	private List<Field> fields = null;

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

	@JsonProperty("constants")
	public List<Constant> getConstants() {
		return constants;
	}

	@JsonProperty("constants")
	public void setConstants(List<Constant> constants) {
		this.constants = constants;
	}

	@JsonProperty("fields")
	public List<Field> getFields() {
		return fields;
	}

	@JsonProperty("fields")
	public void setFields(List<Field> fields) {
		this.fields = fields;
	}

}
