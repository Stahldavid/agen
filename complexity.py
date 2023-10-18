import os
import ast

# Constant folder path for code files
FOLDER_PATH = "/home/stahlubuntu/coder_agent/bd/"

def complexity_analyzer(code_or_filename: str, include_comments: bool = False, include_whitespace: bool = False):
    """
    Analyzes the complexity metrics of a given Python code.

    Parameters:
    - code_or_filename (str): The Python code or filename to be analyzed.
    - include_comments (bool): Whether to include comments in the lines of code count. Default is False.
    - include_whitespace (bool): Whether to include whitespace in the lines of code count. Default is False.

    Returns:
    - dict: A dictionary containing various complexity metrics.
    """
    if code_or_filename.endswith('.py'):
        try:
            with open(os.path.join(FOLDER_PATH, code_or_filename), 'r') as f:
                code = f.read()
        except FileNotFoundError:
            raise ValueError('File not found')
    else:
        code = code_or_filename
    
    tree = ast.parse(code)
    cyclomatic_complexity = 1
    nesting_level = 0
    line_count = code.count('\n') + 1
    parameter_count = 0
    lines_of_comments = 0 if not include_comments else code.count('#')
    lines_of_whitespace = 0 if not include_whitespace else (code.count('\n') - code.count('\n#') - code.count('\n\n') - 1)

    for node in ast.walk(tree):
        if isinstance(node, (ast.If, ast.While, ast.For, ast.With)):
            cyclomatic_complexity += 1
        if isinstance(node, ast.FunctionDef):
            nesting_level += 1
            parameter_count = len(node.args.args)

    lines_of_code = line_count - lines_of_comments - lines_of_whitespace
    
    return {
        'cyclomatic_complexity': cyclomatic_complexity,
        'nesting_level': nesting_level,
        'lines_of_code': lines_of_code,
        'lines_of_comments': lines_of_comments,
        'lines_of_whitespace': lines_of_whitespace,
        'parameter_count': parameter_count
    }

# # Test the function with some sample code
# test_code = """
# def example_function(a, b):
#     if a > b:
#         return a
#     else:
#         return b
# """

# #complexity_analyzer(test_code, include_comments=True, include_whitespace=True)
# print(complexity_analyzer(test_code, include_comments=True, include_whitespace=True))