package it.links.pert.codegen.json;

import java.io.File;
import java.io.IOException;

import com.fasterxml.jackson.core.JsonParseException;
import com.fasterxml.jackson.databind.JsonMappingException;

import it.links.pert.codegen.model.ADF;
import it.links.pert.codegen.model.Function;

public interface ADFReader<E extends Function> {

	ADF<E> read(File adfFile) throws JsonParseException, JsonMappingException, IOException;
}