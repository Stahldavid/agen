functions1=[
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
                                        },
                                        
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
    
]



functions2=[
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
        "name": "read_file",
        "description": "Reads the contents of a file and returns it as a string.",
        "parameters": {
            "type": "object", 
            "properties": {
                "file_path": {
                    "type": "string",
                    "description": "The path of the file to read."
                }
            },
            "required": ["file_path"]
        }
    },
    
    {
        "name": "write_to_file",
        "description": "This function writes the given content to a file at a specified path. It creates the file if it doesn't already exist, and overwrites the file if it does.",
        "parameters": {
            "type": "object",
            "properties": {
                "content": {
                    "type": "string",
                    "description": "The content to be written to the file."
                },
                "file_path": {
                    "type": "string",
                    "description": "The path where the file should be saved. This should include the filename and extension."
                }
            },
            "required": ["content", "file_path"]
        }
    }
]