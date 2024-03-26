
dummy=True

if dummy==True:
    class LLM:
        def __init__(self):
            print("LLM class created")

        def response(self, s):
            return "My response is " + str(s)

else:

    from langchain.prompts.prompt import PromptTemplate
    from langchain.llms import LlamaCpp
    from langchain.chains import ChatVectorDBChain
    import pickle

    class LLM:
        def __init__(self):
            self._llm =LlamaCpp(model_path="./WizardLM-7B-uncensored.ggmlv3.q4_1.bin", temperature=0, n_ctx=2048, verbose=True,  n_gpu_layers=35)
            print("LLM class created (using llm)")

        def response(self, s):
            prompt = """### Instruction: You are an assistant at the Ontario Regiment Museum in Oshawa Ontario. 
               If you don't know the answer, just say "I'm not sure." Don't try to make up an answer.
               Your name is Mary. Use the following pieces of context to answer the user's question. """
            prompt = prompt + f"\n### Input: {s}\n### Response:"
        
            return self._llm(prompt)
