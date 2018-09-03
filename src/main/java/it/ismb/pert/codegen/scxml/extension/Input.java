package it.ismb.pert.codegen.scxml.extension;

import org.apache.commons.scxml2.ActionExecutionContext;
import org.apache.commons.scxml2.SCXMLExpressionException;
import org.apache.commons.scxml2.model.Action;
import org.apache.commons.scxml2.model.ActionExecutionError;
import org.apache.commons.scxml2.model.ModelException;

public class Input extends Action {

	/**
	 * 
	 */
	private static final long serialVersionUID = 1215385273954164582L;
	
	private String type;
	
	public Input() {
		super();
	}

	@Override
	public void execute(ActionExecutionContext arg0)
			throws ModelException, SCXMLExpressionException, ActionExecutionError {
		// TODO Auto-generated method stub

	}

	/**
	 * @return the type
	 */
	public String getType() {
		return type;
	}

	/**
	 * @param type the type to set
	 */
	public void setType(String type) {
		this.type = type;
	}

}
