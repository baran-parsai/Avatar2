###
### Not yet complete
###
from langchain.prompts.prompt import PromptTemplate
from langchain_community.llms import LlamaCpp
from .llm import LLM
import pickle
import time

class LLMLangChain(LLM):
    def __init__(self, model, prompt, vectorstore_name):
        with open(vectorstore_name, "rb") as f:
            self.vectorstore = pickle.load(f)
        self.llm = LlamaCpp(model_path=model, temperature=0, n_ctx=2048, verbose=False,  n_gpu_layers=-1)

    def response(self, text):
        docs = self.vectorstore.as_retriever(search_kwargs={"k":2}).get_relevant_documents(query=text)
        prompt = """You are an assistant at the Ontario Regiment Museum in Oshawa Ontario. 
           If you don't know the answer, just say "I'm not sure." Don't try to make up an answer.
           Your name is Mary. Use the following pieces of context to answer the user's question. """

        for doc in docs:
            prompt = prompt + "\n" + doc.page_content
        prompt = prompt + f"\n### USER: {question}\n### ASSISTANT:"
    
        resp = llm(prompt)
        return resp
