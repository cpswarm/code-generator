package it.links.pert.codegen.model.ros;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;

@JsonInclude(JsonInclude.Include.NON_NULL)
public class Action extends Definition {

	@JsonProperty("goal")
	private Goal goal;
	@JsonProperty("result")
	private Result result;
	@JsonProperty("feedback")
	private Feedback feedback;

	@JsonProperty("goal")
	public Goal getGoal() {
		return goal;
	}

	@JsonProperty("goal")
	public void setGoal(Goal goal) {
		this.goal = goal;
	}

	@JsonProperty("result")
	public Result getResult() {
		return result;
	}

	@JsonProperty("result")
	public void setResult(Result result) {
		this.result = result;
	}

	@JsonProperty("feedback")
	public Feedback getFeedback() {
		return feedback;
	}

	@JsonProperty("feedback")
	public void setFeedback(Feedback feedback) {
		this.feedback = feedback;
	}

}
