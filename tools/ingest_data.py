import argparse
import pickle
from langchain.text_splitter import RecursiveCharacterTextSplitter, CharacterTextSplitter
#from langchain.document_loaders import TextLoader
from langchain_community.document_loaders import TextLoader
from langchain.vectorstores.faiss import FAISS
#from langchain.embeddings import SentenceTransformerEmbeddings
from langchain_community.embeddings import SentenceTransformerEmbeddings


parser = argparse.ArgumentParser(description='Build vector store for a llm')
parser.add_argument('pickle', help='pickle file')
parser.add_argument('text', nargs='*')
parser.add_argument('--verbose', action='store_true', default=False)
args = parser.parse_args()

# Load Data
docs = args.text
all_docs = []
for doc in docs:
    if args.verbose:
        print(f"Loading document {doc}")
    loader = TextLoader(doc)
    all_docs.extend(loader.load())
text_splitter = CharacterTextSplitter(chunk_size=500, separator="\n\n", is_separator_regex=False, length_function=len, chunk_overlap=20)
documents = text_splitter.split_documents(all_docs)

# Load Data to vectorstore
embeddings = SentenceTransformerEmbeddings(model_name="sentence-transformers/all-MiniLM-L6-v2")
if args.verbose:
    print("Embeddings created")
vectorstore = FAISS.from_documents(documents, embeddings)

# Save vectorstore
if args.verbose:
    print(f"Saving pickle file as {args.pickle}")
with open(args.pickle, "wb") as f:
    pickle.dump(vectorstore, f)
