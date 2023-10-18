import os
import openai

from dotenv import load_dotenv
from terminal import terminal_access
from di_code import dynamic_code_execution
from complexity import complexity_analyzer
from profiler import code_profiler
from git_operations import git_operations
from dynamic_web import dynamic_web_scraper
from construct import auto_code
import json
from tenacity import retry, stop_after_attempt, wait_random_exponential
import requests
from termcolor import colored
from code_search import similarity_search
from testt import unit_test_runner
from functions_ca import functions3
from metaphor import metaphor_web_search
from code_reviwer import automated_code_reviewer
from scrape import scrape_web_pages
from utils import file_operations, load_conversation_from_file, save_conversation_to_file, pretty_print_conversation, search_code_completion_request, print_numerated_history, remove_messages_by_indices, ast_tool, pdb_tool, automated_code_reviewer, read_file, read_all_files_in_directory, append_code_to_message
from gpt_api_call import api_call
from tokenizer import num_tokens_from_messages

# Define the GPT models to be used
GPT_MODEL1 = "gpt-3.5-turbo-0613"
GPT_MODEL = "gpt-4-0613"


# Load environment variables from .env file
load_dotenv()

# Access the API key from the environment variable
openai_api_key = os.getenv('OPENAI_API_KEY')
openai.api_key = openai_api_key



# %%



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






max_response_tokens = 1000 # Adjusted to your value
token_limit = 3000  # Adjusted to your value




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
    # {
    #     "role": "user",
    #     "content": f"search for openai functions calling 0613 using methaphor. after, scrape the second link"  
    # }
]
conversation = messages




from tenacity import retry, wait_random_exponential, stop_after_attempt

# @retry(wait=wait_random_exponential(multiplier=1, max=60), stop=stop_after_attempt(15))
# def api_call(messages, functions3, max_response_tokens):
#     try:
#         return openai.ChatCompletion.create(
#             model="gpt-4-0613",
#             messages=messages,
#             functions=functions3,
#             temperature=0.7,
#             max_tokens=max_response_tokens,
#             function_call="auto"
#         )
#     except openai.error.RateLimitError as e:
#         print(f"Rate limit exceeded: {e}")
#         raise
#     except Exception as e:
#         print(f"An unexpected error occurred: {e}")
#         raise



function_mapping = {
    'similarity_search': similarity_search,
    'file_operations': file_operations,
    'ast_tool': ast_tool,
    'pdb_tool': pdb_tool,
    'scrape_web_pages': scrape_web_pages,
    'unit_test_runner': unit_test_runner,
    'automated_code_reviewer': automated_code_reviewer,
    'terminal_access': terminal_access,
    'dynamic_code_execution': dynamic_code_execution,
    'metaphor_web_search': metaphor_web_search,
    'complexity_analyzer': complexity_analyzer,
    'code_profiler': code_profiler,
    'git_operations': git_operations,  # New addition
    'dynamic_web_scraper': dynamic_web_scraper # New addition
}






# while True:
#     user_input = input("Enter message: ")

#     if "@codebase" in user_input or "@" in user_input:
#         directory_path = "/home/stahlubuntu/coder_agent/"

#         # Replace @codebase tag with actual codebase content
#         if "@codebase" in user_input:
#             codebase_str = read_all_files_in_directory(directory_path)
#             user_input = user_input.replace("@codebase", f"{codebase_str}")

#         # Append the content of the files mentioned in the user_input
#         user_input = append_code_to_message(user_input, directory_path)

#     # Append the modified user_input to the conversation
#     conversation.append({"role": "user", "content": user_input})

#     # Check for slash commands
#     if user_input == "/chat_history":
#         pretty_print_conversation(conversation)
#         continue
#     elif user_input.startswith("/clean_chat_history"):
#         if len(user_input) > 19:
#             indices_str = user_input[20:].strip("[]").split(";")
#             try:
#                 indices = [int(idx) for idx in indices_str]
#                 conversation = [msg for idx, msg in enumerate(conversation) if idx not in indices]
#                 print(f"Messages at indices {', '.join(indices_str)} have been removed!")
#             except ValueError:
#                 print("Invalid format. Use /clean_chat_history [index1;index2;...]")
#         else:
#             conversation = []
#             print("Chat history cleared!")
#         continue
#     elif user_input == "/token":
#         tokens = num_tokens_from_messages(conversation)
#         print(f"Current chat history has {tokens} tokens.")
#         continue
#     elif user_input.startswith("/save_conversation"):
#         if len(user_input) > 17:
#             filename = user_input[18:].strip()
#             save_conversation_to_file(conversation, filename)
#         else:
#             print("Invalid format. Use /save_conversation filename")
#         continue
#     elif user_input.startswith("/load_conversation"):
#         if len(user_input) > 17:
#             filename = user_input[18:].strip()
#             conversation = load_conversation_from_file(filename)
#         else:
#             print("Invalid format. Use /load_conversation filename")
#         continue
#     elif user_input == "/help":
#         print("/chat_history - View the chat history")
#         print("/clean_chat_history - Clear the chat history")
#         print("/clean_chat_history [index1;index2;...] - Clear specific messages from the chat history")
#         print("/token - Display the number of tokens in the chat history")
#         print("/save_conversation filename - Save the conversation to a file")
#         print("/load_conversation filename - Load a conversation from a file")
#         print("/print_numerated_history - Print the chat history with each message numbered")
#         print("/remove_messages_by_indices [index1;index2;...] - Remove specific messages from the chat history")
#         print("/help - Display the available slash commands")
#         continue
    
#     conversation.append({"role": "user", "content": user_input})
#     conv_history_tokens = num_tokens_from_messages(conversation)

#     while conv_history_tokens + max_response_tokens >= token_limit:
#         del conversation[1] 
#         conv_history_tokens = num_tokens_from_messages(conversation)

#     while True:
#         chat_response = api_call(conversation, functions3, max_response_tokens)
#         assistant_message = chat_response['choices'][0].get('message')
#         finish_reason = chat_response['choices'][0].get('finish_reason')

#         if assistant_message['content'] is not None:
#             conversation.append({"role": "assistant", "content": assistant_message['content']})
#             pretty_print_conversation(conversation)
#         else:
#             if assistant_message.get("function_call"):
#                 function_name = assistant_message["function_call"]["name"]
#                 arguments = json.loads(assistant_message["function_call"]["arguments"])
                
#                 if function_name in function_mapping:
#                     try:
#                         function_response = function_mapping[function_name](**arguments)
#                     except Exception as e:
#                         function_response = str(e)
                
#                 # Add assistant and function response to conversation
#                 conversation.append({
#                     "role": "assistant",
#                     "function_call": {
#                         "name": function_name,
#                         "arguments": assistant_message["function_call"]["arguments"]
#                     },
#                     "content": None
#                 })
#                 conversation.append({
#                     "role": "function",
#                     "name": function_name,
#                     "content": function_response
#                 })

#                 # Optional: Make a second API call to get the final assistant response
#                 second_response = api_call(conversation, functions3, max_response_tokens)
#                 final_message = second_response["choices"][0]["message"]
#                 if final_message['content'] is not None:
#                     conversation.append({"role": "assistant", "content": final_message['content']})
#                     pretty_print_conversation(conversation)

#         print(f"Finish reason: {finish_reason}")
#         if finish_reason != 'function_call':
#             print("Breaking the inner loop.")
#             break

while True:
    user_input = input("Enter message: ")

    if "@codebase" in user_input or "@" in user_input:
        directory_path = "/home/stahlubuntu/coder_agent/bd"

        # Replace @codebase tag with actual codebase content
        if "@codebase" in user_input:
            codebase_str = read_all_files_in_directory(directory_path)
            user_input = user_input.replace("@codebase", f"{codebase_str}")

        # Append the content of the files mentioned in the user_input
        user_input = append_code_to_message(user_input, directory_path)

    # Append the modified user_input to the conversation
    conversation.append({"role": "user", "content": user_input})

    # Check for slash commands
    if user_input == "/chat_history":
        pretty_print_conversation(conversation)
        continue
    elif user_input.startswith("/clean_chat_history"):
        if len(user_input) > 19:
            indices_str = user_input[20:].strip("[]").split(";")
            try:
                indices = [int(idx) for idx in indices_str]
                conversation = [msg for idx, msg in enumerate(conversation) if idx not in indices]
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
    elif user_input.startswith("/save_conversation"):
        if len(user_input) > 17:
            filename = user_input[18:].strip()
            save_conversation_to_file(conversation, filename)
        else:
            print("Invalid format. Use /save_conversation filename")
        continue
    elif user_input.startswith("/load_conversation"):
        if len(user_input) > 17:
            filename = user_input[18:].strip()
            conversation = load_conversation_from_file(filename)
        else:
            print("Invalid format. Use /load_conversation filename")
        continue
    elif user_input == "/help":
        print("/chat_history - View the chat history")
        print("/clean_chat_history - Clear the chat history")
        print("/clean_chat_history [index1;index2;...] - Clear specific messages from the chat history")
        print("/token - Display the number of tokens in the chat history")
        print("/save_conversation filename - Save the conversation to a file")
        print("/load_conversation filename - Load a conversation from a file")
        print("/print_numerated_history - Print the chat history with each message numbered")
        print("/remove_messages_by_indices [index1;index2;...] - Remove specific messages from the chat history")
        print("/start query - Start the auto_code function with the provided query")
        print("/help - Display the available slash commands")
        continue
    elif user_input.startswith("/start"):
        if len(user_input) > 7:
            query = user_input[8:].strip()
            result = auto_code(query)
            conversation.append({"role": "user", "content": result})
        else:
            print("Invalid format. Use /start query")
        continue
    
    conversation.append({"role": "user", "content": user_input})
    conv_history_tokens = num_tokens_from_messages(conversation)

    while conv_history_tokens + max_response_tokens >= token_limit:
        del conversation[1] 
        conv_history_tokens = num_tokens_from_messages(conversation)

    while True:
        chat_response = api_call(conversation, functions3, max_response_tokens)
        assistant_message = chat_response['choices'][0].get('message')
        finish_reason = chat_response['choices'][0].get('finish_reason')

        if assistant_message['content'] is not None:
            conversation.append({"role": "assistant", "content": assistant_message['content']})
            pretty_print_conversation(conversation)
        else:
            if assistant_message.get("function_call"):
                function_name = assistant_message["function_call"]["name"]
                arguments = json.loads(assistant_message["function_call"]["arguments"])
                
                if function_name in function_mapping:
                    try:
                        function_response = function_mapping[function_name](**arguments)
                    except Exception as e:
                        function_response = str(e)
                
                # Add assistant and function response to conversation
                conversation.append({
                    "role": "assistant",
                    "function_call": {
                        "name": function_name,
                        "arguments": assistant_message["function_call"]["arguments"]
                    },
                    "content": None
                })
                conversation.append({
                    "role": "function",
                    "name": function_name,
                    "content": function_response
                })

                # Optional: Make a second API call to get the final assistant response
                second_response = api_call(conversation, functions3, max_response_tokens)
                final_message = second_response["choices"][0]["message"]
                if final_message['content'] is not None:
                    conversation.append({"role": "assistant", "content": final_message['content']})
                    pretty_print_conversation(conversation)

        print(f"Finish reason: {finish_reason}")
        if finish_reason != 'function_call':
            print("Breaking the inner loop.")
            break