package it.links.pert.codegen.model.ros;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;

@JsonInclude(JsonInclude.Include.NON_NULL)
@JsonPropertyOrder({ "topic", "msg" })
public class Input {

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("topic")
	private String topic;
	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("msg")
	private Msg msg;

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("topic")
	public String getTopic() {
		return topic;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("topic")
	public void setTopic(String topic) {
		this.topic = topic;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("msg")
	public Msg getMsg() {
		return msg;
	}

	/**
	 * 
	 * (Required)
	 * 
	 */
	@JsonProperty("msg")
	public void setMsg(Msg msg) {
		this.msg = msg;
	}

}
