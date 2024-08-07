from langchain.prompts.prompt import PromptTemplate
#import llama-cpp-python
#from langchain_community.llms.llamacpp import LlamaCpp
from langchain.llms import LlamaCpp
from langchain.chains import ChatVectorDBChain
import time
import pickle

_template = """Given the following conversation and a follow up question, rephrase the follow up question to be a standalone question.
You can assume the question about LangChain.

Chat History:
{chat_history}
Follow Up Input: {question}
Standalone question:"""
CONDENSE_QUESTION_PROMPT = PromptTemplate.from_template(_template)

template = """You are an AI assistant for the open source library LangChain. The documentation is located at https://langchain.readthedocs.io.
You are given the following extracted parts of a long document and a question. Provide a conversational answer with a hyperlink to the documentation.
You should only use hyperlinks that are explicitly listed as a source in the context. Do NOT make up a hyperlink that is not listed.
If the question includes a request for code, provide a code block directly from the documentation.
If you don't know the answer, just say "Hmm, I'm not sure." Don't try to make up an answer.
If the question is not about LangChain, politely inform them that you are tuned to only answer questions about LangChain.
LlamaCpp(model_path="../WizardLM-7B-uncensored.Q4_0.gguf",
llm =llama_cpp.Llama(model_path="WizardLM-7B-uncensored.Q4_K_M.gguf", temperature=0, n_ctx=2048, verbose=True,  n_gpu_layers=-1)

Question: {question}
=========
{context}
=========
Answer in Markdown:"""
qa_prompt = PromptTemplate(template=template, input_variables=["question", "context"])

CTX_MAX = 2048
def get_chain(vectorstore):
    global qa_prompt
    llm =LlamaCpp(model_path="WizardLM-7B-uncensored.Q4_K_M.gguf",
                  temperature=0, 
                  n_ctx=1024,
                  n_gpu_layers= -1,  
                  verbose=True, 
                  use_mlock=True
                )
    qa_chain = ChatVectorDBChain.from_llm(
        llm,
        vectorstore,
        combine_docs_chain_kwargs={'prompt': qa_prompt},
        condense_question_prompt=CONDENSE_QUESTION_PROMPT,
    )
    return qa_chain




if __name__ == "__main__":
    question = "When is the clinic open"
    with open("vectorstore_clinic.pkl", "rb") as f:
        vectorstore = pickle.load(f)
    llm =LlamaCpp(model_path="../WizardLM-7B-uncensored.ggmlv3.q4_1.bin", temperature=0, n_ctx=3000, verbose=True,  n_gpu_layers=-1)

    while True:
        print("ASk a queation")
        start_time = time.time()
        question = input()
        docs = vectorstore.as_retriever().get_relevant_documents(query=question)
        print(len(docs))
        sentiment = "neutral"
        print(f"Response->{docs[0].page_content}")
        prompt = f"""### Instruction: You are an assistant at the Exquisite Hearing Clinic . 
           If you don't know the answer, just say "I'm not sure." Don't try to make up an answer.
           Your name is Mary. You can use emojis. Breifly summarize the clinic information. Highlight key points. End with a question to engage the user. Reply in a frinedly, informal tone, as if you are chatting with a friend. Use the following pieces of context to answer the user's question. User who has asked the question has the follwing emotional sentiment: {sentiment} REspond accordingly. Ensure that the response is empathetic and considerate of the user's emotional state."""
#        prompt = prompt + "\n" + docs[0].page_content
        for doc in docs:
            prompt = prompt + "\n" + doc.page_content
        prompt = prompt + f"\n### Input: {question}\n### Response:"
    
        print(prompt)
        print(llm(prompt))
        print(f"Time taken: {time.time() - start_time}")
#    qa_chain = get_chain(vectorstore)
#    chat_history = []
#    print("Chat with your docs!")
#    while True:
#        print("Human:")
#        question = input()
#        result = qa_chain({"question": question, "chat_history": chat_history})
#        chat_history.append((question, result["answer"]))
#        print("AI:")
#        print(result["answer"])
