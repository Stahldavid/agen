

# %%

from definitions import *
import logging
import os
import re
from queue import Queue
from dotenv import load_dotenv
import openai
import json
from functions_ca import functions1, functions2, functions3, functions4
from utils import *
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
GPT_MODEL = "gpt-3.5-turbo-16k-0613"




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
        file_path (str): The path of the file to write to, relative to /bd/.
    """
    with open('/bd/' + file_path, 'w') as f:
        f.write(content)



def read_file(file_path):
    """
    Reads the contents of the file at the given file path.

    Args:
        file_path (str): The path of the file to read from, relative to /bd/.

    Returns:
        str: The contents of the file.
    """
    with open('/bd/' + file_path, 'r') as f:
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




def print_numerated_history(conversation):
    """Prints the chat history with each message prefixed by its number.
    
    Args:
        conversation (list): The list of messages in the conversation.
    """
    for idx, message in enumerate(conversation, 1):
        print(f"{idx}. {message['role'].capitalize()}: {message['content']}")

def remove_messages_by_indices(conversation, indices):
    """Removes messages from the conversation based on the provided indices.
    
    Args:
        conversation (list): The list of messages in the conversation.
        indices (list): The indices of the messages to remove.
        
    Returns:
        list: The updated conversation with messages removed.
    """
    for index in sorted(indices, reverse=True):
        if 0 < index <= len(conversation):
            del conversation[index - 1]
        else:
            print(f"Invalid index: {index}")
    return conversation






import ast
import json
import astor

import ast
import astor

def ast_tool(code, action):
    """
Performs various AST based operations on the provided Python code.

Args:
    code (str): The Python code to analyze. Can be a filename or code string.
    action (dict): The action to perform. Supported actions are:
        - analyze: Analyzes the code and returns extracted information.
        - modify: Modifies names in the AST.
        - generate: Generates code from a provided AST.
        - optimize: Removes specified code from the AST.
        - detect_patterns: Detects specified patterns in the AST.
        - visualize: Returns a string representation of the AST.

Returns:
    str: The result of the requested action on the code. For analyze and 
    detect_patterns this is a dict. For the rest this is the modified code string.
    """
    # Determine if code is a filename or Python code
    if code.endswith('.py'):
        with open(code, 'r') as file:
            code_content = file.read()
    else:
        code_content = code

    # Parse the code into an AST
    tree = ast.parse(code_content)

    # Analyze the code
    if action.get('analyze', {}).get('enabled', False):
        functions = [node.name for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)]
        return {"functions": functions}

    # Modify variable or function names
    if action.get('modify', {}).get('enabled', False):
        old_name = action['modify']['old_name']
        new_name = action['modify']['new_name']
        # Implement renaming logic here
        return astor.to_source(tree)

    # Generate code from AST
    if action.get('generate', {}).get('enabled', False):
        tree_to_generate = action['generate']['tree']
        return astor.to_source(ast.parse(tree_to_generate))

    # Optimize code (remove specified code snippet)
    if action.get('optimize', {}).get('enabled', False):
        code_to_remove = action['optimize']['code_to_remove']
        tree = ast.fix_missing_locations(tree)
        optimized_code = astor.to_source(tree).replace(code_to_remove, '')
        return optimized_code

    # Detect specific patterns or code smells (example: detect print statements)
    if action.get('detect_patterns', False):
        patterns_detected = [node for node in ast.walk(tree) if isinstance(node, ast.Print)]
        return {"print_statements": len(patterns_detected)}

    # Visualize the AST (returning as a string for simplicity)
    if action.get('visualize', False):
        return ast.dump(tree)

    return "Invalid action provided"


import pdb
import json

import json
import pdb
import json

def pdb_tool(json_input):
    """
Uses the Python debugger (pdb) to perform actions on provided code.

Args:
    json_input (str): A JSON string containing the code/filename and action.
        The JSON should have keys:
        - 'code_or_filename': The code string or a filename. 
        - 'action': The debugger action to perform.
        Actions: 
            - 'set_breakpoint': Set a breakpoint at the given line number.
            - 'step': Step through the code line-by-line.
            - 'continue': Continue execution until next breakpoint.
            - 'inspect': Print the value of the given variable.

Returns: 
    str: A message indicating the result of the debugger action.
"""
    # Parse the JSON input
    params = json.loads(json_input)
    code_or_filename = params['code_or_filename']
    action = params['action']

    # Define the folder path
    folder_path = "/home/stahlubuntu/coder_agent/bd/"

    # Check if code input is a filename
    if code_or_filename.endswith('.py'):
        with open(folder_path + code_or_filename, 'r') as file:
            code = file.read()
    else:
        code = code_or_filename

    # Create a PDB instance
    debugger = pdb.Pdb()

    # Handle the action
    if action == 'set_breakpoint':
        line_number = params.get('line_number', None)
        if line_number is None:
            return "Error: line_number is required for 'set_breakpoint' action."
        debugger.set_break(code, line_number)
        result = f"Breakpoint set at line {line_number}"
    elif action == 'step':
        debugger.run(code)
        result = f"Stepping through code: {code}"
    elif action == 'continue':
        debugger.do_continue()
        result = "Continuing execution"
    elif action == 'inspect':
        variable_name = params.get('variable_name', None)
        if variable_name is None:
            return "Error: variable_name is required for 'inspect' action."
        value = debugger.eval(variable_name)
        result = f"Inspecting variable {variable_name}: {value}"

    return result





"""
This code makes a request to the OpenAI Chat Completions API to decompose a complex 
code generation task into smaller sub-tasks. 

It takes in a query string, searches relevant repositories for code snippets using 
that query, and passes those results to the Chat Completions API along with the 
complex task description. 

The API response contains a JSON object with details on each sub-task file needed.
This code processes that JSON to extract the file details into a Python dictionary.

Parameters:
    query (str): The search query string
    results_string (str): The code search results to pass to the API
    functions1 (list): List of functions for the API
    completion (ChatCompletion): The API response

Returns:
    files (dict): Dictionary containing the extracted file details
        Each key is the filename, values are a dict with:
            order (int): Order of development
            code_blocks (list): List of code block dicts in the file
"""


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






"""
Uses the Python debugger (pdb) to perform actions on provided code.

Args:
    json_input (str): A JSON string containing the code/filename and action.
        The JSON should have keys:
        - 'code_or_filename': The code string or a filename. 
        - 'action': The debugger action to perform.
        Actions: 
            - 'set_breakpoint': Set a breakpoint at the given line number.
            - 'step': Step through the code line-by-line.
            - 'continue': Continue execution until next breakpoint.
            - 'inspect': Print the value of the given variable.

Returns: 
    str: A message indicating the result of the debugger action.
"""
# %%

completion2 = openai.ChatCompletion.create(
    model="gpt-3.5-turbo-0613",
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


 
"""
This code takes in a dictionary of file strings (file_strings) and a list of functions 
(functions4). 

It loops through each file string, makes a request to the OpenAI API to optimize the code, 
and saves the optimized code and comments to new dictionaries (new_files and new_comments).

The optimized code and comments are then written out to files in a target directory.

Parameters:
    file_strings (dict): Dictionary with filename keys and file content string values
    functions4 (list): List of functions to pass to the OpenAI API
    
Returns:
    None

Saves:
    Optimized code files to target directory 
    Comment files to target directory
"""

import time
import json
import openai
import os

# Define a string to store the print outputs
output_string = ""
new_files = {}
new_comments = {}  # To store the comments for each file

# Define a target directory to save the files
target_dir = "/home/stahlubuntu/coder_agent/bd"

# Assuming that file_strings and functions4 are defined elsewhere in your code

# Loop through each file_string in the file_strings dictionary
for filename, file_string in file_strings.items():
    wait_time = 1
    max_attempts = 5
    attempts = 0

    while attempts < max_attempts:
        try:
            completion4 = openai.ChatCompletion.create(
                model="gpt-4-0613",
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
                functions=functions4,
                function_call={"name": "optimize_code"}
            )

            # Extracting data from completion4
            reply_content4 = completion4.choices[0]
            args4 = reply_content4["message"]['function_call']['arguments']
            data4 = json.loads(args4)

            # Save the optimized code and comments
            new_files[filename] = data4['code']
            new_comments[filename] = data4['comments']

            # Append to the output_string
            output_string += f"For file: {filename}, the improved code is: {new_files[filename]}\n"

            break
        except openai.error.OpenAIError as e:
            print(f"Encountered an error: {e}. Retrying in {wait_time} seconds...")
            time.sleep(wait_time)
            attempts += 1
            wait_time *= 2

# Ensure the target directory exists
if not os.path.exists(target_dir):
    os.makedirs(target_dir)

# Loop through the filenames and corresponding code and comments in new_files and new_comments
for filename, code_content in new_files.items():
    comments_content = new_comments.get(filename, '')

    # Define the full file paths for the code and comments files
    code_file_path = os.path.join(target_dir, filename)
    comments_file_path = os.path.join(target_dir, f"{filename}_comments.txt")

    # Save the code content to the code file
    with open(code_file_path, "w") as code_file:
        code_file.write(code_content)

    # Save the comments content to the comments file
    with open(comments_file_path, "w") as comments_file:
        comments_file.write(comments_content)

print("Files and comments saved successfully!")










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

def pretty_print_conversation(messages):
    """
Prints a conversation between a system, user, and assistant in a readable format.

The conversation is passed in as a list of message dictionaries. Each message has a
"role" (system, user, assistant) and "content". 

This function loops through the messages and prints them with the role name and  
content. The text is color coded based on the role using the role_to_color mapping.

Special handling is done for messages with the "function" role to print the function
name and output.

Parameters:
    messages (list): The list of message dictionaries representing the conversation.
        Each message dict contains "role" and "content" keys.
        
Returns:
    None
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





import openai
import tiktoken
import json

# Get the CL100KBase encoding and create a Tokenizer instance
cl100k_base = tiktoken.get_encoding("cl100k_base")
tokenizer = cl100k_base

max_response_tokens = 2500
token_limit = 7000  # Adjusted to your value
#conversation = []
#conversation.append(system_message)

def num_tokens_from_messages(messages):
    """
Calculates the number of tokens used so far in a conversation.

This uses the CL100KBase tokenizer to encode the conversation messages
and tally up the total number of tokens.

Each message is encoded as:
<im_start>{role/name}\n{content}<im_end>\n

Parameters:
    messages (list): The list of conversation messages. 
        Each message is a dict with "role" and "content" keys.
        
Returns:
    num_tokens (int): The total number of tokens used in the conversation.
"""
    encoding = tiktoken.get_encoding("cl100k_base")  # Model to encoding mapping
    num_tokens = 0
    for message in messages:
        num_tokens += 4  # Every message follows <im_start>{role/name}\n{content}<im_end>\n
        for key, value in message.items():
            if isinstance(value, str):  # Ensure value is a string before encoding
                num_tokens += len(encoding.encode(value))
                if key == "name":  # If there's a name, the role is omitted
                    num_tokens -= 1  # Role is always required and always 1 token
    num_tokens += 2  # Every reply is primed with <im_start>assistant
    return num_tokens




"""
This code defines a conversational agent that can analyze and improve code snippets.

It initializes the conversation with an introductory system message and a user
message containing the code snippets to analyze. 

It then enters a loop to collect user input messages, check for slash commands,
and pass the conversation to the OpenAI Chat Completions API if it is under the
token limit.

The num_tokens_from_messages() function calculates the number of tokens used so far.

If the API response contains an assistant message, it is printed and added to the
conversation. If it contains a function call, that function is executed and the 
output is printed.

The similarity_search() function can be called to fetch relevant code snippets.
This function is defined in functions3 as:

similarity_search(query, directories) - Vectorstore embedding semantic search for code functions. It receives a query and the directories to search, and returns the most similar code snippets to the queries.

read_file(file_path) - Reads the contents of a file and returns it as a string.

write_to_file(content, file_path) - Writes given content to a specified file path, overwriting existing files.

pdb_tool(code_or_filename, action, line_number, variable_name) - Interacts with the Python debugger (PDB) to debug code.

Parameters:
    conversation (list): The conversation history
    functions3 (list): Helper functions for the API, including similarity_search
    max_response_tokens (int): Max tokens for the API response 
    token_limit (int): Max tokens allowed overall

Returns:
    None
    
The code improves the provided code snippets based on the conversation.

"""

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
conversation = messages






while True:
    user_input = input("Enter message: ")  
    
    # Check for slash commands
    if user_input == "/chat_history":
        print_numerated_history(conversation)
        continue
    elif user_input.startswith("/clean_chat_history"):
        if len(user_input) > 19:
            indices_str = user_input[20:].strip("[]").split(";")
            try:
                indices = [int(idx) for idx in indices_str]
                conversation = remove_messages_by_indices(conversation, indices)
                print(f"Messages at indices {', '.join(indices_str)} have been removed!")
            except ValueError:
                print("Invalid format. Use /clean_chat_history [index1;index2;...]")
        else:
            conversation = []
            print("Chat history cleared!")
        continue
    elif user_input == "/token":
        tokens = num_tokens_from_messages(conversation)
        print(f"Current chat history has {tokens} tokens.")
        continue
    elif user_input == "/help":
        print("/chat_history - View the chat history (numerated)")
        print("/clean_chat_history - Clear the chat history")
        print("/clean_chat_history [index1;index2;...] - Clear specific messages from the chat history")
        print("/token - Display the number of tokens in the chat history")
        print("/help - Display the available slash commands")
        continue
    
    conversation.append({"role": "user", "content": user_input})
    conv_history_tokens = num_tokens_from_messages(conversation)

    while conv_history_tokens + max_response_tokens >= token_limit:
        del conversation[1] 
        conv_history_tokens = num_tokens_from_messages(conversation)

    chat_response = openai.ChatCompletion.create(
        model="gpt-4-0613",
        messages=conversation,
        functions=functions3,
        temperature=0.7,
        max_tokens=max_response_tokens,
    )

    assistant_message = chat_response['choices'][0].get('message')
    if assistant_message['content'] is not None:
        conversation.append({"role": "assistant", "content": assistant_message['content']})
        pretty_print_conversation(conversation)
    else:
        if assistant_message.get("function_call"):
            function_name = assistant_message["function_call"]["name"]
            arguments = json.loads(assistant_message["function_call"]["arguments"])

            if function_name == "similarity_search":
                results = similarity_search(arguments['query'], arguments['directories'])
                function_message = {
                    "role": "function",
                    "name": function_name,
                    "content":  f"code search content: {results}"
                }
                conversation.append(function_message)
                pretty_print_conversation(conversation)
            elif function_name == "write_to_file":
                write_to_file(arguments['content'], arguments['file_path'])
                function_message = {
                    "role": "function",
                    "name": function_name,
                    "content": f"File successfully written at {arguments['file_path']}"
                }
                conversation.append(function_message)
                pretty_print_conversation(conversation)
            elif function_name == "read_file":
                content = read_file(arguments['file_path'])
                function_message = {
                    "role": "function",
                    "name": function_name,
                    "content": content
                }
                conversation.append(function_message)
                pretty_print_conversation(conversation)
            elif function_name == "ast_tool":
                result = ast_tool(**arguments)
                function_message = {
                    "role": "function",
                    "name": function_name,
                    "content": f"Result: {result}, Input Arguments: {arguments}"
                }
                conversation.append(function_message)
                pretty_print_conversation(conversation)
            elif function_name == "pdb_tool":
                result = pdb_tool(**arguments)
                function_message = {
                    "role": "function",
                    "name": function_name,
                    "content": result
                }
                conversation.append(function_message)
                pretty_print_conversation(conversation)



