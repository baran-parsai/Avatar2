from langchain.prompts.prompt import PromptTemplate
from langchain.llms import LlamaCpp
from langchain.chains import ConversationalRetrievalChain
from langchain.memory import ConversationBufferMemory
import pickle



if __name__ == "__main__":
    memory = ConversationBufferMemory(memory_key="chat_history", return_messages=True)
    with open("vectorstore.pkl", "rb") as f:
        vectorstore = pickle.load(f)
    llm = LlamaCpp(model_path="./WizardLM-7B-uncensored.ggmlv3.q4_1.bin", temperature=0, n_ctx=2048, verbose=False,  n_gpu_layers=35)
    qa = ConversationalRetrievalChain.from_llm(llm, vectorstore.as_retriever(), return_source_documents=True, verbose=False)


    chat_history = []
    while True:
        print(">", end="")
        question = input()
        docs = vectorstore.as_retriever().get_relevant_documents(query=question)
        prompt = """### Instruction: You are a helpful assistant named Mary at the Ontario Regiment Museum in Oshawa Ontario.
           If you don't know the answer, just say "I'm not sure." Don't try to make up an answer.
           """

        prompt = prompt + f"\n### Input: {question}\n### Response:"
    
#        print(prompt)
        result = qa({"question":prompt, "chat_history":chat_history})
        print(result['answer'])
#        print(result)
        chat_history = [(question, result["answer"])]
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
