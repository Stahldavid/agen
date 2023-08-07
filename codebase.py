

# %%
import logging
import os
import re
from queue import Queue
from dotenv import load_dotenv
import openai
import json
from functions_ca import functions1, functions2, functions3
from langchain.chat_models import ChatOpenAI
from langchain.chains import LLMChain
from langchain.prompts import (
    ChatPromptTemplate,
    SystemMessagePromptTemplate,
    HumanMessagePromptTemplate,
)

# Load environment variables from .env file
load_dotenv()

# Access the API key from the environment variable
openai_api_key = os.getenv('OPENAI_API_KEY')
openai.api_key = openai_api_key
# Set up logging
logging.basicConfig(level=logging.INFO)



import json
import openai
from code_search import similarity_search

query = "Variable impedance control for force feedback"
results_string = similarity_search(query)
print(results_string)


completion = openai.ChatCompletion.create(
    model="gpt-3.5-turbo-16k-0613",
    messages=[
        {
            "role": "system",
            "content": "You are an AI that decomposes complex code generation tasks into smaller, manageable sub-tasks. Each sub-task should be a independent file, should contain the name of the python file, and should contain the name of the file,description, all the functions and classes from the file, as well releted files."
        },
        {
            "role": "user",
            "content": f"Given the complex code generation task: 'Write a variable impedance control for force feedback using ros2, webots, webots_ros2 and ros2_control.', please decompose it into a detailed, numbered list of sub-tasks. Each sub-task should be a independent file should contain the name of the python file, description,all the functions and classes from the file, as well releted files. Make sure to devide the task into minimum 5 files. Try to make the code as readable as possible, encapsulating the code into functions and classes. Lets think step by step.\n\nThe following are the retrived documents from all the relevant repositories based on the query 'Variable impedance control for force feedback':\n{results_string}\nThese retrived functions from all the relevant repositories are helpfull but not fullfile the user task. Please use this context to help guide the task decompositionThese retrived functions from all the relevant repositories are helpfull but not fullfile the user task. Please use this context to help guide the task decomposition"
        }
    ],
    functions=functions1, 
    function_call={"name": "generate_code"}
)

reply_content = completion.choices[0]
print(reply_content)

args = reply_content["message"]['function_call']['arguments']
data = json.loads(args)

# Initialize an empty dictionary to store the files
files = {}

# Go through each file
for file in data["files"]:
    # Create a new dictionary for this file
    files[file["code_blocks"][0]["name"]] = {
        "order": file["order"],
        "code_blocks": file["code_blocks"],
    }

# Sort the files dictionary based on the order of development
files = dict(sorted(files.items(), key=lambda item: item[1]['order']))

# Print the files dictionary
for filename, file_data in files.items():
    print(f"Order of development: {file_data['order']}")
    print(f"{filename}:")
    for block in file_data['code_blocks']:
        print(f"  Code block type: {block['type']}")
        print(f"  Code block name: {block['name']}")
        print(f"  Code block description: {block['description']}")
        print(f"  Code block content: {block['content']}")
        #print(f"  Related files: {block['related_files']}")


files_string = ""
for filename, file_data in files.items():
    files_string += f"Order of development: {file_data['order']}\n"
    files_string += f"{filename}:\n"
    for block in file_data['code_blocks']:
        files_string += f"  Code block type: {block['type']}\n"
        files_string += f"  Code block name: {block['name']}\n"
        files_string += f"  Code block description: {block['description']}\n"
        files_string += f"  Code block content: {block['content']}\n"
        #files_string += f"  Related files: {block['related_files']}\n"

# %%

completion2 = openai.ChatCompletion.create(
    model="gpt-4-0613",
    messages=[
        {
            "role": "system",
            "content": "You are an advanced AI with capabilities to analyze intricate code and pseudocode files. Based on this analysis, you provide recommendations for the most appropriate vectorstore repositories to extract relevant code snippets from. In addition, you generate search queries that could be utilized to fetch these helpful code samples."
        },
        {
            "role": "user",
            "content": f"I need your expertise to examine the provided code and pseudocode files. Your task is to pinpoint any issues, inefficiencies, and areas for potential enhancements. Here are the files you need to look into:\n\n{files_string}"
        }
    ],
    functions = functions2,
    function_call={"name": "analyze_code"}

    )
    


reply_content2 = completion2.choices[0]
print(reply_content2)

args2 = reply_content2["message"]['function_call']['arguments']
data2 = json.loads(args2)


print(data2)


# Define the list of directories to search in
directories = ['db_ros2_control', 'db_ros2', 'db_webots_ros2', 'db_webots']

# Create an empty dictionary to store the results
results = {}

# Loop through each file in the data2 dictionary
for file_data in data2["files"]:
    file_name = file_data["file_name"]
    
    repository = file_data["repository"]
    query = repository['query']
    
    # Call the similarity_search function and save the result as a string
    result = similarity_search(query, directories)
    
    # Store the result in the dictionary, using the filename_query as the key
    results[f"{file_name}_{query}"] = result

# Create a dictionary to store the strings for each file
file_strings = {}

# Loop through each file in the files dictionary
for filename, file_data in files.items():
    # Create a list to store the lines for this file
    file_lines = []
    file_lines.append(f"Order of development: {file_data['order']}")
    file_lines.append(f"{filename}:")
    
    for block in file_data['code_blocks']:
        file_lines.append(f"  Code block type: {block['type']}")
        file_lines.append(f"  Code block name: {block['name']}")
        file_lines.append(f"  Code block description: {block['description']}")
        file_lines.append(f"  Code block content: {block['content']}")
        
        # Loop through the results dictionary to find the results for this file
        for key, value in results.items():
            # If the filename is in the key of the results dictionary, add the query and its result to the lines
            if filename in key:
                file_lines.append(f"#####################################")
                file_lines.append(f"  Query:\n\n {key.replace(filename+'_', '')}")
                file_lines.append(f"\n\n  Query result:\n\n {value}")
    
    # Join the lines for this file into a single string and add it to the file_strings dictionary
    file_strings[filename] = '\n'.join(file_lines)


# %%
# Loop through each file_string in the file_strings dictionary
for filename, file_string in file_strings.items():
    print(f"File: {filename}")
    print(file_string)

# %%
# Loop through each file_string in the file_strings dictionary
for filename, file_string in file_strings.items():
    print(f"File: {filename}")
    print(file_string)

# %%
# Define a string to store the print outputs
output_string = ""

# Loop through each file_string in the file_strings dictionary
for filename, file_string in file_strings.items():
    # Create a new completion with the file_string as the user message content
    completion4 = openai.ChatCompletion.create(
        model="gpt-4",
        messages=[
            {
                "role": "system",
                "content": "You are an AI Code Optimization Model that can optimize code, complete #TODOs, recommend best practices, and learn from relevant code repositories. Your main task is to analyze the given code, which performs semantic search queries on a vectorstore using OpenAI embeddings, and apply the insights from the search results to refine the code. The final goal is to produce a fully functional and optimized code file that can be used as part of a larger project, specifically a variable impedance control code for force feedback"
            },
            {
                "role": "user",
                "content": f"I am working on a coding project that aims to develop a variable impedance control code for force feedback. I need your expertise to improve my code. Here is the current version of one file of my code, along with the semantic search queries I’ve done on a vectorstore using OpenAI embeddings, and the results of these queries:\n\n{file_string}\n\nCan you improve this code, using the suggestions from the semantic search results? Please write the improved and complete code file. Please complete and improve the file based on the context."
            }
        ],
    )

    new_files = {}
    new_files[filename] = completion4.choices[0].message['content']
    # Append to the output_string instead of printing
    output_string += f"For file: {filename}, the improved code is: {new_files[filename]}\n"

# Now you can print or further process the output_string as required
print(output_string)
    # Print or process the completion as needed
    #print(f"For file: {filename}, the improved code is: {new_files[filename]}\n")


# %%
print(output_string)

# %%
import json
import openai
import requests
from tenacity import retry, wait_random_exponential, stop_after_attempt
import os
import json
import requests
from termcolor import colored
from dotenv import load_dotenv
from code_search import similarity_search
import openai

# Define the GPT models to be used
GPT_MODEL1 = "gpt-3.5-turbo-0613"
GPT_MODEL = "gpt-4-0613"

# Load environment variables from .env file
load_dotenv()

# Access the API key from the environment variable
openai_api_key = os.getenv('OPENAI_API_KEY')
openai.api_key = openai_api_key

def chat_completion_request(messages, functions=None, function_call=None, model=GPT_MODEL):
    """
    Makes a request to the OpenAI Chat Completions API.

    Args:
        messages (list): A list of messages in the conversation.
        functions (dict): A dictionary of functions to be used in the conversation.
        function_call (dict): A dictionary containing the function call to be made.
        model (str): The name of the GPT model to be used.

    Returns:
        dict: The response from the API.
    """
    headers = {
        "Content-Type": "application/json",
        "Authorization": "Bearer " + openai.api_key,
    }
    json_data = {"model": model, "messages": messages}
    if functions is not None:
        json_data.update({"functions": functions})
    if function_call is not None:
        json_data.update({"function_call": function_call})
    try:
        response = requests.post(
            "https://api.openai.com/v1/chat/completions",
            headers=headers,
            json=json_data,
        )
        response.raise_for_status()
        return response.json()
    except Exception as e:
        print("Unable to generate ChatCompletion response")
        print(f"Exception: {e}")
        return e

def write_to_file(content, file_path):
    """
    Writes the given content to the file at the given file path.

    Args:
        content (str): The content to write to the file.
        file_path (str): The path of the file to write to.
    """
    with open(file_path, 'w') as f:
        f.write(content)


def read_file(file_path):
    """
    Reads the contents of the file at the given file path.

    Args:
        file_path (str): The path of the file to read from.

    Returns:
        str: The contents of the file.
    """
    with open(file_path, 'r') as f:
        return f.read()

def pretty_print_conversation(messages):
    """
    Prints the conversation in a pretty format.

    Args:
        messages (list): A list of messages in the conversation.
    """
    role_to_color = {
        "system": "red",
        "user": "green",
        "assistant": "blue",
        "function": "magenta",
    }
    for message in messages:
        color = role_to_color.get(message["role"], "white")
        if message["role"] == "function":
            print(colored(f'{message["role"]}: {message["name"]} output: {message["content"]}', color))
        else:
            print(colored(f'{message["role"]}: {message["content"]}', color))

def search_code_completion_request(messages, functions):
    """
    Makes a request to the OpenAI Chat Completions API for code search.

    Args:
        messages (list): A list of messages in the conversation.
        functions (dict): A dictionary of functions to be used in the conversation.

    Returns:
        dict: The response from the API.
    """
    return openai.ChatCompletion.create(
        model="gpt-3.5-turbo-16k-0613",
        messages=messages,
        functions=functions,
        function_call={"auto"}
    )

# Define the initial messages in the conversation
functions3 = functions3
messages = [
    {
        "role": "system",
        "content": "You are a sophisticated AI that has the ability to analyze complex code and pseudocode documents. You are tasked with making necessary clarifications in a series of chat turns until you gather sufficient information to rewrite the code. You can utilize the 'search_code' function to fetch relevant code snippets based on semantic similarity, and subsequently improve the given file. After each search you should improve the file, do not make several calls to the function before improving the file."
    },
    {
        "role": "user",
        "content": f"I need your assistance in reviewing these code and pseudocode documents. You final goal is to rewrite and finish the code to make full functional The final goal is to create a project for variable impedance control providing force feedback. The project will use Webots, ROS2, webots_ros2, and ros2_control. You are required to identify potential problems, inefficiencies, and areas for improvements in these documents. Here are the documents you need to work on:\n\n{output_string}\n\nPlease first clarify any question that you need to finish the code with me. After you completely understand the goal of the user, use the search_code function to find relevant code that can help improve the code."  
    }
]

import openai
import tiktoken
import json

# Get the CL100KBase encoding and create a Tokenizer instance
cl100k_base = tiktoken.get_encoding("cl100k_base")
tokenizer = cl100k_base

# # Define the function to count tokens in a list of messages
# def num_tokens_from_messages(messages):
#     num_tokens = 0
#     for message in messages:
#         num_tokens += len(tokenizer.encode(message["content"]))  # Add 4 for special tokens
#     num_tokens += 2 # Add 2 for <im_start>assistant which always primes a user's message
#     return num_tokens


def num_tokens_from_messages(messages):
    encoding = tiktoken.get_encoding("cl100k_base")
    tokens = []
    for message in messages:
        tokens += encoding.encode(message["content"])  # Corrected this line
    return len(tokens)



# Start the chat
token_limit = 7000 # Adjust the token limit to 7000
#messages = []
while True:
    user_input = input("Enter message: ")
    user_message = {
        "role": "user",
        "content": user_input
    }
    messages.append(user_message)
    model_messages = messages.copy() # Create a copy of the messages for the model

    while num_tokens_from_messages(messages) > token_limit:
        print(f"Exceeding token limit of {token_limit}. Removing earliest messages until below 5000 tokens.")
        while num_tokens_from_messages(messages) > token_limit - 2000: # Continue removing messages until we reach 5000 tokens
            removed_message = messages.pop(0) # Remove the earliest message
            print(f"Removed message: {removed_message['content']}") # Print the removed message

    chat_response = openai.ChatCompletion.create(
        model="gpt-4-0613",
        messages=model_messages,
        functions=functions3,
    )

    assistant_message = chat_response['choices'][0].get('message')
    if assistant_message:
        messages.append(assistant_message)
        pretty_print_conversation(messages)

        if assistant_message.get("function_call"):
            function_name = assistant_message["function_call"]["name"]
            arguments = json.loads(assistant_message["function_call"]["arguments"])

            if function_name == "similarity_search":
                results = similarity_search(arguments['query'], arguments['directories'])
                function_message = {
                    "role": "function",
                    "name": function_name,
                    "content": results
                }
                messages.append(function_message)
                pretty_print_conversation(messages)
            elif function_name == "write_to_file":
                write_to_file(arguments['content'], arguments['file_path'])
                function_message = {
                    "role": "function",
                    "name": function_name,
                    "content": f"File successfully written at {arguments['file_path']}"
                }
                messages.append(function_message)
                pretty_print_conversation(messages)
