package it.links.pert.codegen.model.ros;

import com.fasterxml.jackson.annotation.JsonInclude;
import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.annotation.JsonPropertyOrder;

@JsonInclude(JsonInclude.Include.NON_NULL)
@JsonPropertyOrder({
    "topic",
    "msg_def"
})
public class Output {

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
    @JsonProperty("msg_def")
    private RosMsg RosMsg;

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
    @JsonProperty("msg_def")
    public RosMsg getMsgDef() {
        return RosMsg;
    }

    /**
     * 
     * (Required)
     * 
     */
    @JsonProperty("msg_def")
    public void setMsgDef(RosMsg msgDef) {
        this.RosMsg = msgDef;
    }

}
