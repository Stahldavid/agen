functions1 = [
    {
        "name": "generate_code",
        "description": "Generates the code for multiple files, each described by a dictionary of attributes.",
        "parameters": {
            "type": "object",
            "properties": {
                "files": {
                    "type": "array",
                    "description": "An array of dictionaries, each representing a file. Each dictionary should include 'order' (the order of development), 'code_blocks' (an array of dictionaries detailing the code blocks in the file).",
                    "items": {
                        "type": "object",
                        "properties": {
                            "order": {
                                "type": "integer",
                                "description": "The order of development for the file."
                            },
                            "code_blocks": {
                                "type": "array",
                                "description": "An array of dictionaries, each detailing a code block in the file. Each dictionary should include 'type' (either 'function' or 'class'), 'name' (the name of the function or class), 'description' (a description of the block's purpose), 'content' (the details of the function or class, including function arguments or class methods, as applicable), and 'related_files' (an array of filenames that are related to the code block).",
                                "items": {
                                    "type": "object",
                                    "properties": {
                                        "type": {
                                            "type": "string",
                                            "description": "The type of the code block, either 'function' or 'class'."
                                        },
                                        "name": {
                                            "type": "string",
                                            "description": "The name of the function or class."
                                        },
                                        "description": {
                                            "type": "string",
                                            "description": "A description of the block's purpose."
                                        },
                                        "content": {
                                            "type": "string",
                                            "description": "The details of the function or class, including arguments and methods as applicable."
                                        }
                                    },
                                    "required": ["type", "name", "description", "content"]
                                }
                            }
                        },
                        "required": ["order", "code_blocks"]
                    }
                }
            },
            "required": ["files"]
        }
    }
]

functions2 = [
    {
        "name": "analyze_code",
        "description": "This function performs an analysis on the provided code files. It returns a list of suitable repositories for fetching relevant code samples and suggests a respective search query for each repository.",
        "parameters": {
            "type": "object",
            "properties": {
                "files": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "file_name": {
                                "type": "string",
                                "description": "The name of the code file."
                            },
                            "repository": {
                                "type": "object",
                                "properties": {
                                    "name": {
                                        "type": "string",
                                        "description": "The name of the repository.",
                                        "enum": ['db_ros2_control', 'db_ros2', 'db_webots_ros2', 'db_webots']
                                    },
                                    "query": {
                                        "type": "string",
                                        "description": "The search query designed to fetch code samples from the specified repository."
                                    }
                                },
                                "required": ["name", "query"],
                                "description": "An object representing a repository and a corresponding search query."
                            }
                        },
                        "required": ["file_name", "repository"]
                    },
                    "description": "An array of objects, each representing a code file that needs to be analyzed."
                }
            },
            "required": ["files"]
        }
    }
]

functions3 = [
    {
        "name": "similarity_search",
        "description": "Vectorstore embedding semantic search for code functions. It receives a query and the directories to search, and returns the most similar code snippets to the queries.",
        "parameters": {
            "type": "object",
            "properties": {
                "query": {
                    "type": "string",
                    "description": "The search query designed to fetch code samples."
                },
                "directories": {
                    "type": "array",
                    "items": {
                        "type": "string",
                        "enum": ["db_webots", "db_ros2", "db_webots_ros2", "db_ros2_control"],
                        "description": "The directories in which to perform the search."
                    },
                    "description": "An array of directory names."
                }
            },
            "required": ["query", "directories"]
        }
    },
   


{
    "name": "pdb_tool",
    "description": "A tool for interacting with the Python Debugger (PDB). Allows setting breakpoints, stepping through code, continuing execution, and inspecting variables.",
    "parameters": {
        "type": "object",
        "properties": {
            "code_or_filename": {
                "type": "string",
                "description": "The Python code to be debugged or the filename of the Python file to be debugged."
            },
            "debug_operations": {
                "type": "object",
                "properties": {
                    "set_breakpoints": {
                        "type": "array",
                        "items": {
                            "type": "integer"
                        },
                        "description": "List of line numbers for setting breakpoints."
                    },
                    "step": {
                        "type": "boolean",
                        "description": "Whether to step through the code."
                    },
                    "continue": {
                        "type": "boolean",
                        "description": "Whether to continue execution."
                    },
                    "inspect": {
                        "type": "array",
                        "items": {
                            "type": "string"
                        },
                        "description": "List of variable names to inspect."
                    }
                }
            }
        },
        "required": ["code_or_filename", "debug_operations"]
    }
},
 
    {
        "name": "ast_tool",
        "description": "Analyzes the Abstract Syntax Tree (AST) of a given Python code to provide insights such as function calls, variable assignments, and control flow structures.",
        "parameters": {
            "type": "object",
            "properties": {
                "code_or_filename": {
                    "type": "string",
                    "description": "The Python code to be analyzed or the filename of the Python file to be analyzed."
                },
                "analyze_functions": {
                    "type": "boolean",
                    "description": "Whether to analyze function calls in the code."
                },
                "analyze_variables": {
                    "type": "boolean",
                    "description": "Whether to analyze variable assignments in the code."
                },
                "analyze_control_flow": {
                    "type": "boolean",
                    "description": "Whether to analyze control flow structures like loops and conditionals in the code."
                }
            },
            "required": ["code_or_filename"]
        }
    },

{
    "name": "automated_code_reviewer",
    "description": "Automatically reviews the given code and provides feedback in a JSON-formatted string using pylint.",
    "parameters": {
        "type": "object",
        "properties": {
            "code_or_filename": {
                "type": "string",
                "description": "The Python code as a string or the filename of the Python file to be reviewed."
            }
        },
        "required": ["code_or_filename"]
    }
},

{
    "name": "metaphor_web_search",
    "description": "Searches the web for articles based on a given query using the Metaphor API. ",
    "parameters": {
        "type": "object",
        "properties": {
            "query": {
                "type": "string",
                "description": "The search query. The search query designed to fetch code samples. Rephrase questions to look more like answers.For example, instead of searching for “What’s the best way to get started with cooking?”, try rephrase the question to resemble an answer;'This is the best tutorial on how to get started with cooking:'"
            },
         
            "start_published_date": {
                "type": "string",
                "description": "The start date for when the document was published (in YYYY-MM-DD format)."
            },
            "end_published_date": {
                "type": "string",
                "description": "The end date for when the document was published (in YYYY-MM-DD format)."
            }
        },
        "required": ["query"]
    }
},

# {
#     "name": "scrape_web_pages",
#     "description": "Scrapes HTML content from a list of web pages.",
#     "parameters": {
#         "type": "object",
#         "properties": {
#             "urls": {
#                 "type": "array",
#                 "items": {
#                     "type": "string",
#                     "description": "The URL of the web page to scrape."
#                 },
#                 "description": "A list of URLs to scrape."
#             }
#         },
#         "required": ["urls"]
#     }
# },
{
    "name": "file_operations",
    "description": "Performs various file operations such as read, write, copy, move, and mkdir.",
    "parameters": {
        "type": "object",
        "properties": {
            "operation_type": {
                "type": "string",
                "enum": ["read", "write", "copy", "move", "mkdir"],
                "description": "The type of file operation to perform."
            },
            "source_path": {
                "type": "string",
                "description": "The path to the source file or directory."
            },
            "destination_path": {
                "type": "string",
                "description": "The destination path for write, copy, move, or mkdir operations."
            },
            "content": {
                "type": "string",
                "description": "The content to be written to the file."
            }
        },
        "required": ["operation_type", "source_path"]
    }
},


{
    "name": "code_profiler",
    "description": "Profile the given Python code using cProfile and returns the profiling data sorted by the given criteria.",
    "parameters": {
        "type": "object",
        "properties": {
            "code_or_filename": {
                "type": "string",
                "description": "The Python code or filename to be profiled as a string."
            },
            "sort_by": {
                "type": "string",
                "enum": ["cumulative", "time", "calls", "name"],
                "default": "cumulative",
                "description": "The criteria for sorting the profiling data. Default is 'cumulative'."
            },
            "limit": {
                "type": "integer",
                "default": 10,
                "description": "The number of lines to limit the output to. Default is 10."
            }
        },
        "required": ["code_or_filename"]
    }
},

{
    "name": "git_operations",
    "description": "Handle various Git operations.",
    "parameters": {
        "type": "object",
        "properties": {
            "operation_type": {
                "type": "string",
                "enum": ["clone", "checkout", "commit", "push", "pull", "init", "status", "log", "add_remote", "remove_remote", "fetch", "reset", "tag"],
                "description": "The type of git operation."
            },
            "repo_path": {
                "type": "string",
                "description": "The repository path where the git operation will be performed. Optional; defaults to the current working directory."
            },
            "branch_name": {
                "type": "string",
                "description": "The branch name."
            },
            "commit_message": {
                "type": "string",
                "description": "The commit message."
            },
            "files": {
                "type": "array",
                "items": {
                    "type": "string"
                },
                "description": "List of files to commit."
            },
            "remote_name": {
                "type": "string",
                "description": "The name of the remote."
            },
            "remote_url": {
                "type": "string",
                "description": "The URL of the remote repository."
            },
            "tag_name": {
                "type": "string",
                "description": "The name of the tag."
            },
            "commit_sha": {
                "type": "string",
                "description": "The commit SHA for the reset operation."
            }
        },
        "required": ["operation_type"]
    }
},



{
  "name": "unit_test_runner",
  "description": "Runs unit tests on a given Python code or file and returns the results.",
  "parameters": {
    "type": "object",
    "properties": {
      "code_or_filename": {
        "type": "string",
        "description": "The Python code as a string or the filename of the Python file to be tested."
      },
      "test_code_or_filename": {
        "type": "string",
        "description": "The test Python code as a string or the filename of the test Python file."
      }
    },
    "required": ["code_or_filename", "test_code_or_filename"]
  }
},

{
    "name": "terminal_access",
    "description": "This function runs a given Bash command on the terminal and returns the output. Use with extreme caution.",
    "parameters": {
        "type": "object",
        "properties": {
            "command": {
                "type": "string",
                "description": "The Bash command to be executed."
            }
        },
        "required": ["command"]
    }
},
{
    "name": "dynamic_web_scraper",
    "description": "Scrapes content from a dynamic web page using Playwright. Supports multiple actions like navigation, click, scrolling, and waiting for elements.",
    "parameters": {
        "type": "object",
        "properties": {
            "url": {
                "type": "string",
                "description": "The URL of the web page to scrape."
            },
            "actions": {
                "type": "array",
                "items": {
                    "type": "object",
                    "properties": {
                        "type": {
                            "type": "string",
                            "enum": ["navigate_browser", "previous_page", "click_element", "scroll_down", "wait_for_element", "extract_text", "extract_hyperlinks", "current_page"],
                            "description": "The type of action to perform."
                        },
                        "value": {
                            "type": "string",
                            "description": "Any value associated with the action, such as a CSS selector for clicking or waiting."
                        }
                    },
                    "required": ["type"]
                },
                "description": "An array of actions to perform on the web page before scraping."
            }
        },
        "required": ["url", "actions"]
    }
},




    {
    "name": "dynamic_code_execution",
    "description": "Compiles and executes Python code dynamically, allowing optional inputs and return variables.",
    "parameters": {
        "type": "object",
        "properties": {
            "code_or_filename": {
                "type": "string",
                "description": "The Python code or filename to compile and execute."
            },
            "mode": {
                "type": "string",
                "enum": ["exec", "eval", "single"],
                "description": "Mode in which the code should be executed. Can be 'exec', 'eval', or 'single'."
            },
            "input_vars": {
                "type": "object",
                "description": "Optional dictionary of variables to be used in the code execution."
            },
            "return_vars": {
                "type": "array",
                "items": {
                    "type": "string"
                },
                "description": "Optional list of variable names to return after code execution."
            }
        },
        "required": ["code_or_filename"]
    }
},

{
    "name": "complexity_analyzer",
    "description": "Analyzes a given Python code to calculate various code metrics such as cyclomatic complexity, nesting level, lines of code, lines of comments, lines of whitespace, and parameter count.",
    "parameters": {
        "type": "object",
        "properties": {
            "code_or_filename": {
                "type": "string",
                "description": "The Python code or filename that needs to be analyzed."
            },
            "include_cyclomatic_complexity": {
                "type": "boolean",
                "description": "Whether to include cyclomatic complexity in the analysis."
            },
            "include_nesting_level": {
                "type": "boolean",
                "description": "Whether to include nesting level in the analysis."
            },
            "include_line_metrics": {
                "type": "boolean",
                "description": "Whether to include line-based metrics (lines of code, comments, whitespace) in the analysis."
            },
            "include_parameter_count": {
                "type": "boolean",
                "description": "Whether to include parameter count in the analysis."
            }
        },
        "required": ["code_or_filename"]
    }
}




]

functions4 = [
    {
        "name": "optimize_code",
        "description": "This function optimizes the given code by applying insights from semantic search results using OpenAI embeddings. It aims to produce a fully functional and optimized code file for a specific project, such as variable impedance control code for force feedback.",
        "parameters": {
            "type": "object",
            "properties": {
                "code": {
                    "type": "string",
                    "description": "The code that needs to be optimized."
                },
                "comments": {
                    "type": "string",
                    "description": "Any additional comments or context related to the code."
                }
            },
            "required": ["code", "comments"]
        }
    }
]





# while True:
#     user_input = input("Enter message: ")  
    
#     # Check for slash commands
#     if user_input == "/chat_history":
#         print_numerated_history(conversation)
#         continue
#     elif user_input.startswith("/clean_chat_history"):
#         if len(user_input) > 19:
#             indices_str = user_input[20:].strip("[]").split(";")
#             try:
#                 indices = [int(idx) for idx in indices_str]
#                 conversation = remove_messages_by_indices(conversation, indices)
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
#     elif user_input == "/help":
#         print("/chat_history - View the chat history (numerated)")
#         print("/clean_chat_history - Clear the chat history")
#         print("/clean_chat_history [index1;index2;...] - Clear specific messages from the chat history")
#         print("/token - Display the number of tokens in the chat history")
#         print("/help - Display the available slash commands")
#         continue
    
#     conversation.append({"role": "user", "content": user_input})
#     conv_history_tokens = num_tokens_from_messages(conversation)

#     while conv_history_tokens + max_response_tokens >= token_limit:
#         del conversation[1] 
#         conv_history_tokens = num_tokens_from_messages(conversation)

#     chat_response = openai.ChatCompletion.create(
#         model="gpt-4-0613",
#         messages=conversation,
#         functions=functions3,
#         temperature=0.7,
#         max_tokens=max_response_tokens,
#     )

#     assistant_message = chat_response['choices'][0].get('message')
#     if assistant_message['content'] is not None:
#         conversation.append({"role": "assistant", "content": assistant_message['content']})
#         pretty_print_conversation(conversation)
#     else:
#         if assistant_message.get("function_call"):
#             function_name = assistant_message["function_call"]["name"]
#             arguments = json.loads(assistant_message["function_call"]["arguments"])

#             if function_name == "similarity_search":
#                 results = similarity_search(arguments['query'], arguments['directories'])
#                 function_message = {
#                     "role": "function",
#                     "name": function_name,
#                     "content":  f"code search content: {results}"
#                 }
#                 conversation.append(function_message)
#                 pretty_print_conversation(conversation)
#             elif function_name == "write_to_file":
#                 write_to_file(arguments['content'], arguments['file_path'])
#                 function_message = {
#                     "role": "function",
#                     "name": function_name,
#                     "content": f"File successfully written at {arguments['file_path']}"
#                 }
#                 conversation.append(function_message)
#                 pretty_print_conversation(conversation)
#             elif function_name == "read_file":
#                 content = read_file(arguments['file_path'])
#                 function_message = {
#                     "role": "function",
#                     "name": function_name,
#                     "content": content
#                 }
#                 conversation.append(function_message)
#                 pretty_print_conversation(conversation)
#             elif function_name == "ast_tool":
#                 analyze_functions = arguments.get('analyze_functions', False)
#                 analyze_variables = arguments.get('analyze_variables', False)
#                 analyze_control_flow = arguments.get('analyze_control_flow', False)
#                 ast_result = ast_tool(arguments['code'], analyze_functions, analyze_variables, analyze_control_flow)
#                 function_message = {US
#                     "role": "function",
#                     "name": function_name,
#                     "content": ast_result
#                 }
#                 conversation.append(function_message)
#                 pretty_print_conversation(conversation)

#             elif function_name == "pdb_tool":
#                 # Get code_or_filename argument
#                 code_or_filename = arguments['code_or_filename']

#                 # Initialize debug_operations dictionary
#                 debug_operations = arguments['debug_operations']

#                 # Check if set_breakpoints exists within debug_operations
#                 if 'set_breakpoints' in debug_operations:
#                     debug_operations["set_breakpoints"] = debug_operations["set_breakpoints"]

#                 # Check if step exists within debug_operations
#                 if 'step' in debug_operations:
#                     debug_operations["step"] = debug_operations['step']

#                 # Check if continue exists within debug_operations
#                 if 'continue' in debug_operations:
#                     debug_operations["continue"] = debug_operations['continue']

#                 # Check if inspect exists within debug_operations
#                 if 'inspect' in debug_operations:
#                     debug_operations["inspect"] = debug_operations['inspect']

#                 # Call pdb_tool function
#                 result = pdb_tool(code_or_filename, debug_operations)

#                 function_message = {
#                     "role": "function",
#                     "name": function_name,
#                     "content": result
#                 }

#                 conversation.append(function_message)
#                 pretty_print_conversation(conversation)
