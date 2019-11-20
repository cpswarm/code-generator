package it.links.pert.codegen.model.ros;

import java.io.IOException;

import com.fasterxml.jackson.core.JsonParser;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.core.TreeNode;
import com.fasterxml.jackson.databind.DeserializationContext;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.deser.std.StdDeserializer;

public class CommModelDeserializer extends StdDeserializer<CommModel> {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1L;

	public CommModelDeserializer() {
		this(null);
	}

	protected CommModelDeserializer(Class<?> vc) {
		super(vc);
	}

	@Override
	public CommModel deserialize(JsonParser parser, DeserializationContext ctxt)
			throws IOException, JsonProcessingException {

		final TreeNode node = parser.getCodec().readTree(parser);
		final ObjectMapper mapper = (ObjectMapper) parser.getCodec();
		final String paradigm = ((JsonNode) node.get("paradigm")).asText();
		if ("rosaction".contentEquals(paradigm)) {
			final Definition action = mapper.treeToValue(node.get("definition"), Action.class);
			return new CommModel(CommModel.Paradigm.fromValue(paradigm), action);
		} else if("rosservice".contentEquals(paradigm)) {
			final Definition action = mapper.treeToValue(node.get("definition"), Service.class);
			return new CommModel(CommModel.Paradigm.fromValue(paradigm), action);	
		}
		return null;
	}

}
