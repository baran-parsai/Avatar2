from .llm import LLM

class LLMDummy(LLM):
    def __init__(self):
        print("Dummy LLM class created")

    def response(self, text):
        print(f"returning My response is {str(text)}")
        return "My response is " + str(text)
