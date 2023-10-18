# import json
# import os

# def dynamic_code_execution(code_or_filename, mode="exec", input_vars=None, return_vars=None):
#     """
#     Compiles and executes Python code dynamically.

#     Parameters:
#         code_or_filename (str): The Python code or filename to compile and execute.
#         mode (str): Mode in which the code should be executed ("exec", "eval", "single").
#         input_vars (dict): Optional dictionary of variables to be used in the code execution.
#         return_vars (list): Optional list of variable names to return after code execution.

#     Returns:
#         str: A string containing the values of the specified return variables or the result of the eval expression.
#     """

#     output = {}
#     result = None

#     # Folder path for code files
#     folder_path = "/home/stahlubuntu/coder_agent/bd/"  # Change this to the directory where your .py files are stored
#         # Constant folder path for code files
    

#     # Check if code input is a filename
#     if code_or_filename.endswith('.py'):
#         try:
#             with open(os.path.join(folder_path, code_or_filename), 'r') as file:
#                 code_str = file.read()
#             output["info"] = f"Reading file {code_or_filename}"
#         except FileNotFoundError:
#             output["error"] = "File not found"
#             return json.dumps(output)
#     else:
#         code_str = code_or_filename
#         output["info"] = "Executing code snippet"
    
#     if input_vars is not None:
#         locals().update(input_vars)

#     try:
#         compiled_code = compile(code_str, "<string>", mode)
        
#         if mode == "exec":
#             exec(compiled_code)
#         elif mode == "eval":
#             result = eval(compiled_code)
#         elif mode == "single":
#             exec(compiled_code)
#         else:
#             output["error"] = "Invalid mode specified."
#             return json.dumps(output)
            
#     except Exception as e:
#         output["error"] = str(e)
#         return json.dumps(output)

#     if mode == "eval":
#         output["result"] = result

#     if return_vars is not None:
#         for var in return_vars:
#             try:
#                 output[var] = locals()[var]
#             except KeyError:
#                 output[var] = "Variable not found."

#     return str(output)

import json
import os

def dynamic_code_execution(code_or_filename: str, mode: str = "exec", input_vars: dict = None, return_vars: list = None):
    """
    Compiles and executes Python code dynamically.

    Parameters:
        code_or_filename (str): The Python code or filename to compile and execute.
        mode (str): Mode in which the code should be executed ("exec", "eval", "single"). Default is "exec".
        input_vars (dict): Optional dictionary of variables to be used in the code execution. Default is None.
        return_vars (list): Optional list of variable names to return after code execution. Default is None.

    Returns:
        str: A string containing the output of the code execution, including any error messages.
    """

    output = {}
    
    # Folder path for code files
    folder_path = "/home/stahlubuntu/coder_agent/bd/"

    # Check if code input is a filename
    if code_or_filename.endswith('.py'):
        try:
            with open(os.path.join(folder_path, code_or_filename), 'r') as file:
                code_str = file.read()
            output["info"] = f"Reading file {code_or_filename}"
        except FileNotFoundError:
            output["error"] = "File not found"
            return json.dumps(output)
    else:
        code_str = code_or_filename
        output["info"] = "Executing code snippet"

    if input_vars:
        locals().update(input_vars)

    try:
        compiled_code = compile(code_str, "<string>", mode)

        if mode == "exec":
            exec(compiled_code)
        elif mode == "eval":
            output["result"] = eval(compiled_code)
        elif mode == "single":
            exec(compiled_code)
        else:
            output["error"] = "Invalid mode specified."
            return json.dumps(output)

    except Exception as e:
        output["error"] = str(e)
        return json.dumps(output)

    if return_vars:
        for var in return_vars:
            try:
                output[var] = locals()[var]
            except KeyError:
                output[var] = "Variable not found."

    return json.dumps(output)
