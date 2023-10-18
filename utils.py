
from definitions import *

import os
import ast
import openai
import pdb
import io
from contextlib import redirect_stdout
import json 
import subprocess
from dotenv import load_dotenv
from terminal import terminal_access
from di_code import dynamic_code_execution
from complexity import complexity_analyzer
from profiler import code_profiler
import tiktoken

# Load environment variables from .env file
load_dotenv()

# Access the API key from the environment variable
openai_api_key = os.getenv('OPENAI_API_KEY')
openai.api_key = openai_api_key


import os
import shutil

import os
import shutil

def file_operations(operation_type: str, source_path: str, destination_path: str = None, content: str = None):
    """
    Performs various file operations such as read, write, copy, and move.

    Parameters:
    - operation_type: The type of file operation ('read', 'write', 'copy', 'move')
    - source_path: The source file path, relative to /home/stahlubuntu/coder_agent/bd/
    - destination_path: The destination file path, relative to /home/stahlubuntu/coder_agent/bd/ (optional)
    - content: The content to write to the file (optional, only for 'write')

    Returns:
    - A string containing the file content for 'read', or a confirmation message for other operations.
    """
    # Set default values for optional parameters
    if destination_path is None:
        destination_path = ''
    if content is None:
        content = ''
    
    base_path = '/home/stahlubuntu/coder_agent/bd/'
    source_full_path = os.path.join(base_path, source_path)
    
    if operation_type == 'read':
        with open(source_full_path, 'r') as f:
            return f.read()

    elif operation_type == 'write':
        destination_full_path = os.path.join(base_path, destination_path) if destination_path else source_full_path
        with open(destination_full_path, 'w') as f:
            f.write(content)
        return f"Content written to {destination_full_path}"

    elif operation_type == 'copy':
        destination_full_path = os.path.join(base_path, destination_path)
        shutil.copy(source_full_path, destination_full_path)
        return f"File copied from {source_full_path} to {destination_full_path}"

    elif operation_type == 'move':
        destination_full_path = os.path.join(base_path, destination_path)
        shutil.move(source_full_path, destination_full_path)
        return f"File moved from {source_full_path} to {destination_full_path}"

    else:
        return "Invalid operation_type. Supported operations are 'read', 'write', 'copy', 'move'."

# Example usages
# print(file_operations('read', 'example.txt'))
# print(file_operations('write', 'example.txt', content='Hello, world!'))
# print(file_operations('copy', 'example.txt', 'example_copy.txt'))
# print(file_operations('move', 'example_copy.txt', 'example_moved.txt'))


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

def save_conversation_to_file(conversation, filename):
    """Saves the conversation to a file.
    
    Args:
        conversation (list): The list of messages in the conversation.
        filename (str): The name of the file to save the conversation to.
        
    Returns:
        None
    """
    with open(filename, 'w') as f:
        for message in conversation:
            f.write(f"{message['role'].capitalize()}: {message['content']}\n")
    print(f"Conversation saved to {filename}.")

def load_conversation_from_file(filename):
    """Loads a conversation from a file.
    
    Args:
        filename (str): The name of the file to load the conversation from.
        
    Returns:
        list: The conversation loaded from the file.
    """
    conversation = []
    with open(filename, 'r') as f:
        for line in f:
            role, content = line.strip().split(': ')
            conversation.append({'role': role.lower(), 'content': content})
    print(f"Conversation loaded from {filename}.")
    return conversation





# def ast_tool(code: str, analyze_functions: bool = False, analyze_variables: bool = False, analyze_control_flow: bool = False) -> str:
#     """
#     Analyzes the Abstract Syntax Tree (AST) of a given Python code.
    
#     Parameters:
#     - code (str): The Python code that needs to be analyzed.
#     - analyze_functions (bool): Whether to analyze function calls in the code.
#     - analyze_variables (bool): Whether to analyze variable assignments in the code.
#     - analyze_control_flow (bool): Whether to analyze control flow structures like loops and conditionals.
    
#     Returns:
#     - str: A string representation of a dictionary containing the analysis results.
#     """
    
#     # Parse the code into an AST
#     tree = ast.parse(code)
    
#     # Initialize results dictionary
#     results = {
#         'functions': [],
#         'variables': [],
#         'control_flow': []
#     }
    
#     # Walk through the AST nodes
#     for node in ast.walk(tree):
#         if analyze_functions and isinstance(node, ast.Call):
#             results['functions'].append(node.func.id if hasattr(node.func, 'id') else str(node.func))
#         if analyze_variables and isinstance(node, ast.Assign):
#             for target in node.targets:
#                 if isinstance(target, ast.Name):
#                     results['variables'].append(target.id)
#         if analyze_control_flow:
#             if isinstance(node, ast.If):
#                 results['control_flow'].append('if_statement')
#             elif isinstance(node, ast.For):
#                 results['control_flow'].append('for_loop')
#             elif isinstance(node, ast.While):
#                 results['control_flow'].append('while_loop')
    
#     # Convert the results dictionary to a string
#     results_str = json.dumps(results, indent=4)
    
#     return results_str

# def ast_tool(code_or_filename: str, analyze_functions: bool = False, analyze_variables: bool = False, analyze_control_flow: bool = False) -> str:
#     """
#     Analyzes the Abstract Syntax Tree (AST) of a given Python code.
    
#     Parameters:
#     - code_or_filename (str): The Python code or filename that needs to be analyzed.
#     - analyze_functions (bool): Whether to analyze function calls in the code.
#     - analyze_variables (bool): Whether to analyze variable assignments in the code.
#     - analyze_control_flow (bool): Whether to analyze control flow structures like loops and conditionals.
    
#     Returns:
#     - str: A string representation of a dictionary containing the analysis results.
#     """
#     folder_path = "/home/stahlubuntu/coder_agent/bd/"

#     # Read the code from file if a filename is provided
#     if code_or_filename.endswith('.py'):
#         with open(folder_path + code_or_filename, 'r') as file:
      
#             code = file.read()
#     else:
#         code = code_or_filename
    
#     # Parse the code into an AST
#     tree = ast.parse(code)
    
#     # Initialize results dictionary
#     results = {
#         'functions': [],
#         'variables': [],
#         'control_flow': []
#     }
    
#     # Walk through the AST nodes
#     for node in ast.walk(tree):
#         if analyze_functions and isinstance(node, ast.Call):
#             results['functions'].append(node.func.id if hasattr(node.func, 'id') else str(node.func))
#         if analyze_variables and isinstance(node, ast.Assign):
#             for target in node.targets:
#                 if isinstance(target, ast.Name):
#                     results['variables'].append(target.id)
#         if analyze_control_flow:
#             if isinstance(node, ast.If):
#                 results['control_flow'].append('if_statement')
#             elif isinstance(node, ast.For):
#                 results['control_flow'].append('for_loop')
#             elif isinstance(node, ast.While):
#                 results['control_flow'].append('while_loop')
    
#     # Convert the results dictionary to a string
#     results_str = json.dumps(results, indent=4)
    
#     return results_str

def ast_tool(code_or_filename: str) -> str:
    folder_path = "/home/stahlubuntu/coder_agent/bd/"

    if code_or_filename.endswith('.py'):
        with open(folder_path + code_or_filename, 'r') as file:
            code = file.read()
    else:
        code = code_or_filename

    tree = ast.parse(code)

    results = {
        'functions': [],
        'variables': [],
        'control_flow': [],
        'imports': [],
        'classes': [],
        'defined_functions': [],
        'error_handling': []
    }

    for node in ast.walk(tree):
        # Analyzing imports
        if isinstance(node, (ast.Import, ast.ImportFrom)):
            import_detail = {'module': ''}
            if isinstance(node, ast.Import):
                import_detail['module'] = node.names[0].name
            else:
                import_detail['module'] = node.module
                import_detail['imported_names'] = [n.name for n in node.names]
            results['imports'].append(import_detail)

        # Analyzing function calls
        if isinstance(node, ast.Call):
            func_detail = {'name': '', 'arguments': []}
            if hasattr(node.func, 'id'):
                func_detail['name'] = node.func.id
            elif isinstance(node.func, ast.Attribute):
                func_detail['name'] = node.func.attr
            func_detail['arguments'] = [ast.dump(arg) for arg in node.args]
            results['functions'].append(func_detail)

        # Analyzing variable assignments
        if isinstance(node, ast.Assign):
            var_detail = {'name': '', 'value': '', 'type': '', 'operations': []}
            for target in node.targets:
                if isinstance(target, ast.Name):
                    var_detail['name'] = target.id
                    var_detail['value'] = ast.dump(node.value)
                    var_detail['type'] = type(node.value).__name__
                    results['variables'].append(var_detail)

        # Analyzing control flow structures
        if isinstance(node, (ast.If, ast.For, ast.While)):
            flow_detail = {'type': '', 'details': ast.dump(node)}
            flow_detail['type'] = type(node).__name__
            results['control_flow'].append(flow_detail)

        # Analyzing class definitions
        if isinstance(node, ast.ClassDef):
            class_detail = {'name': node.name, 'methods': []}
            for n in node.body:
                if isinstance(n, ast.FunctionDef):
                    class_detail['methods'].append(n.name)
            results['classes'].append(class_detail)

        # Analyzing function definitions
        if isinstance(node, ast.FunctionDef):
            func_detail = {'name': node.name, 'arguments': [arg.arg for arg in node.args.args]}
            results['defined_functions'].append(func_detail)


        # Analyzing try-except blocks
        if isinstance(node, ast.Try):
            try_detail = {'type': 'try_except', 'body': [ast.dump(n) for n in node.body], 'handlers': [ast.dump(n) for n in node.handlers]}
            results['error_handling'].append(try_detail)



    results_str = json.dumps(results, indent=4)
    return results_str



# def pdb_tool(code_or_filename: str, debug_operations: dict) -> str:
#     # Initialize the result messages list
#     result_messages = []
    
#     # Folder path for code files
#     folder_path = "/home/stahlubuntu/coder_agent/bd/"
    
#     # Validate code_or_filename
#     if not isinstance(code_or_filename, str):
#         raise ValueError("code_or_filename must be a string")
    
#     # Check if code input is a filename
#     if code_or_filename.endswith('.py'):
#         with open(folder_path + code_or_filename, 'r') as file:
#             code = file.read()
#         result_messages.append(f"Reading file {code_or_filename}")
#     else:
#         code = code_or_filename
#         result_messages.append(f"Debugging code snippet")
    
#     result_messages.append(f"Debugging target: {code_or_filename}")

#     # Prepare PDB commands
#     pdb_commands = []

#     # Validate and set breakpoints
#     if "set_breakpoints" in debug_operations:
#         for bp in debug_operations["set_breakpoints"]:
#             if not isinstance(bp, int):
#                 raise ValueError("Each breakpoint must be an integer")
#             pdb_commands.append(f"b {bp}")
#             result_messages.append(f"Breakpoint set at line {bp}")

#     # Validate and step through code
#     if "step" in debug_operations and debug_operations["step"]:
#         if not isinstance(debug_operations["step"], bool):
#             raise ValueError("Step operation must be a boolean")
#         pdb_commands.append("s")
#         result_messages.append("Stepping through code")

#     # Validate and continue execution
#     if "continue" in debug_operations and debug_operations["continue"]:
#         if not isinstance(debug_operations["continue"], bool):
#             raise ValueError("Continue operation must be a boolean")
#         pdb_commands.append("c")
#         result_messages.append("Continuing execution")

#     # Validate and inspect variables
#     if "inspect" in debug_operations:
#         for var in debug_operations["inspect"]:
#             if not isinstance(var, str):
#                 raise ValueError("Each variable to inspect must be a string")
#             pdb_commands.append(f"p {var}")
#             result_messages.append(f"Inspecting variable {var}")

#     # Run the PDB session with prepared commands
#     with io.StringIO() as buffer, redirect_stdout(buffer):
#         debugger = pdb.Pdb(stdin=io.StringIO("\n".join(pdb_commands)), stdout=buffer)
#         debugger.run(code)

#         # Append any PDB output
#         pdb_output = buffer.getvalue()
#         if pdb_output:
#             result_messages.append(f"PDB Output:\n{pdb_output}")

#     return "\n".join(result_messages)


import io
import pdb
from contextlib import redirect_stdout

def pdb_tool(code_or_filename: str, debug_operations: dict) -> str:
    # Initialize the result messages list
    result_messages = []
    
    # Folder path for code files
    folder_path = "/home/stahlubuntu/coder_agent/bd/"
    
    # Validate code_or_filename
    if not isinstance(code_or_filename, str):
        raise ValueError("code_or_filename must be a string")
    
    # Check if code input is a filename
    if code_or_filename.endswith('.py'):
        with open(folder_path + code_or_filename, 'r') as file:
            code = file.read()
        result_messages.append(f"Reading file {code_or_filename}")
    else:
        code = code_or_filename
        result_messages.append(f"Debugging code snippet")
    
    result_messages.append(f"Debugging target: {code_or_filename}")

    # Prepare PDB commands
    pdb_commands = []

    # Validate and set breakpoints
    if debug_operations.get("set_breakpoints"):
        for bp in debug_operations["set_breakpoints"]:
            if not isinstance(bp, int):
                raise ValueError("Each breakpoint must be an integer")
            pdb_commands.append(f"b {bp}")
            result_messages.append(f"Breakpoint set at line {bp}")

    # Validate and step through code
    if debug_operations.get("step"):
        if not isinstance(debug_operations["step"], bool):
            raise ValueError("Step operation must be a boolean")
        pdb_commands.append("s")
        result_messages.append("Stepping through code")

    # Validate and continue execution
    if debug_operations.get("continue"):
        if not isinstance(debug_operations["continue"], bool):
            raise ValueError("Continue operation must be a boolean")
        pdb_commands.append("c")
        result_messages.append("Continuing execution")

    # Validate and inspect variables
    if debug_operations.get("inspect"):
        for var in debug_operations["inspect"]:
            if not isinstance(var, str):
                raise ValueError("Each variable to inspect must be a string")
            pdb_commands.append(f"p {var}")
            result_messages.append(f"Inspecting variable {var}")

    # Run the PDB session with prepared commands
    with io.StringIO() as buffer, redirect_stdout(buffer):
        debugger = pdb.Pdb(stdin=io.StringIO("\n".join(pdb_commands)), stdout=buffer)
        debugger.run(code)

        # Append any PDB output
        pdb_output = buffer.getvalue()
        if pdb_output:
            result_messages.append(f"PDB Output:\n{pdb_output}")

    return "\n".join(result_messages)



def automated_code_reviewer(code_or_filename):
    """
    Perform an automated code review using Pylint and return all categories as a string.

    Parameters:
        code_or_filename (str): The Python code or filename to review.

    Returns:
        str: String-formatted review feedback.
    """
    
    # Initialize the review_feedback dictionary
    review_feedback = {}
    
    # Constant folder path for code files
    FOLDER_PATH = "/home/stahlubuntu/coder_agent/bd/"

    try:
        # Check if code input is a filename
        if code_or_filename.endswith('.py'):
            with open(f"{FOLDER_PATH}{code_or_filename}", 'r') as file:
                code = file.read()
        else:
            code = code_or_filename

        # Save code to a temporary file to run pylint
        with open("temp_code.py", "w") as temp_file:
            temp_file.write(code)

        # Run pylint on the code and capture its output
        pylint_output = subprocess.getoutput(f"pylint temp_code.py")

        return pylint_output
    
    except FileNotFoundError:
        return "Error: File not found."
    except Exception as e:
        return f"An error occurred: {e}"


def read_file(file_path):
    """
    Reads the contents of the file at the given file path.

    Args:
        file_path (str): The path of the file to read from, relative to /home/stahlubuntu/coder_agent/bd/.

    Returns:
        str: The contents of the file.
    """
    with open('/home/stahlubuntu/coder_agent/bd/' + file_path, 'r') as f:
        return f.read()





def read_all_files_in_directory(directory_path):
    all_files_content = []
    for file_name in os.listdir(directory_path):
        file_path = os.path.join(directory_path, file_name)
        if os.path.isfile(file_path):
            with open(file_path, 'r') as f:
                file_content = f.read()
            all_files_content.append(f"\n--- BEGIN {file_name} ---\n{file_content}\n--- END {file_name} ---")
    return ''.join(all_files_content)



def append_code_to_message(message, directory_path):
    words = message.split()
    file_names = [word[1:] for word in words if word.startswith("@")]
    appended_code = []

    for file_name in file_names:
        file_path = f"{directory_path}/{file_name}"
        file_content = read_file(file_path)

        if file_content is not None:
            appended_code.append(f"\n--- BEGIN {file_name} ---\n{file_content}\n--- END {file_name} ---")
        else:
            print(f"File {file_name} not found.")

    return f"{message}{''.join(appended_code)}"




