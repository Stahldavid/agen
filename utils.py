
from definitions import *


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
    """Print the chat history with each message prefixed by its number."""
    for idx, message in enumerate(conversation, 1):
        print(f"{idx}. {message['role'].capitalize()}: {message['content']}")

def remove_messages_by_indices(conversation, indices):
    """Remove messages from the conversation based on provided indices."""
    for index in sorted(indices, reverse=True):
        if 0 < index <= len(conversation):
            del conversation[index - 1]
        else:
            print(f"Invalid index: {index}")
    return conversation












import ast
import unittest

# Compile code logic
def compile_code(source_code, compile_mode):
    return compile(source_code, '<string>', compile_mode)

# Debug code logic
def debug_code(source_code):
    parsed_ast = ast.parse(source_code)
    debug_info = []
    def find_unused_variables(node):
        assignments = [n.targets[0].id for n in ast.walk(node) if isinstance(n, ast.Assign)]
        references = [n.id for n in ast.walk(node) if isinstance(n, ast.Name)]
        unused_variables = set(assignments) - set(references)
        if unused_variables:
            debug_info.append(f"Unused variables found: {', '.join(unused_variables)}")
    def function_complexity(node):
        if isinstance(node, ast.FunctionDef):
            branches = sum(1 for n in ast.walk(node) if isinstance(n, (ast.If, ast.For, ast.While)))
            debug_info.append(f"Function '{node.name}' has a complexity of {branches} branches.")
    for node in ast.walk(parsed_ast):
        find_unused_variables(node)
        function_complexity(node)
    return debug_info if debug_info else "No issues found."

# Optimize code logic
def optimize_code(source_code):
    parsed_ast = ast.parse(source_code)
    optimizer = CodeOptimizer()
    optimized_ast = optimizer.visit(parsed_ast)
    return ast.unparse(optimized_ast) if hasattr(ast, 'unparse') else "ast.unparse not available."

class CodeOptimizer(ast.NodeTransformer):
    def __init__(self):
        self.called_functions = set()
        self.constant_vars = {}
        self.used_imports = set()

    # 1: Removal of Dead Code
    def visit_Call(self, node):
        if isinstance(node.func, ast.Name):
            self.called_functions.add(node.func.id)
        return self.generic_visit(node)

    def visit_FunctionDef(self, node):
        if node.name not in self.called_functions:
            return None
        return self.generic_visit(node)

    # 2: Expression Simplification
    def visit_BinOp(self, node):
        self.generic_visit(node)
        if isinstance(node.op, ast.Mult) and isinstance(node.right, ast.Num) and node.right.n == 1:
            return node.left
        if isinstance(node.op, ast.Add) and isinstance(node.right, ast.Num) and node.right.n == 0:
            return node.left
        return node

    # 5: Constant Variable Substitution
    def visit_Assign(self, node):
        if len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and isinstance(node.value, ast.Num):
            self.constant_vars[node.targets[0].id] = node.value.n
        return self.generic_visit(node)

    def visit_Name(self, node):
        if node.id in self.constant_vars:
            return ast.Num(self.constant_vars[node.id])
        return self.generic_visit(node)

    # 6: Import Optimization
    def visit_Import(self, node):
        for name in node.names:
            if name.name not in self.used_imports:
                return None
        return node

    def visit_ImportFrom(self, node):
        for name in node.names:
            if name.name not in self.used_imports:
                return None
        return node

    # 4: Unrolling Small Loops
    def visit_For(self, node):
        if isinstance(node.iter, ast.Call) and isinstance(node.iter.func, ast.Name) and node.iter.func.id == "range":
            if isinstance(node.iter.args[0], ast.Num) and node.iter.args[0].n <= 5:
                return node.body * node.iter.args[0].n
        return self.generic_visit(node)




# Analyze code logic
def analyze_code(source_code, options=None):
    parsed_ast = ast.parse(source_code)
    analysis_results = []
    def find_unused_variables(node):
        assignments = [n.targets[0].id for n in ast.walk(node) if isinstance(n, ast.Assign)]
        references = [n.id for n in ast.walk(node) if isinstance(n, ast.Name)]
        unused_variables = set(assignments) - set(references)
        if unused_variables:
            analysis_results.append(f"Unused variables found: {', '.join(unused_variables)}")
    def function_complexity(node):
        if isinstance(node, ast.FunctionDef):
            branches = sum(1 for n in ast.walk(node) if isinstance(n, (ast.If, ast.For, ast.While)))
            analysis_results.append(f"Function '{node.name}' has a complexity of {branches} branches.")
    for node in ast.walk(parsed_ast):
        find_unused_variables(node)
        function_complexity(node)
    return analysis_results if analysis_results else "No issues found."

# Test code logic
def test_code(test_class):
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(test_class)
    runner = unittest.TextTestRunner()
    result = runner.run(suite)
    return str(result)

# Visualize AST logic
def visualize_code(source_code):
    parsed_ast = ast.parse(source_code)
    def visualize_node(node, indent=0):
        children = list(ast.iter_child_nodes(node))
        description = f"{' ' * indent}{node.__class__.__name__}"
        if children:
            description += " -> [" + ", ".join(visualize_node(child, indent + 2) for child in children) + "]"
        return description
    visualization = visualize_node(parsed_ast)
    return visualization
import pdb
# Unparse AST logic
def unparse_ast(source_code):
    parsed_ast = ast.parse(source_code)
    return ast.unparse(parsed_ast) if hasattr(ast, 'unparse') else "ast.unparse not available."

def pdb_tool(source_code, action, breakpoints=None, step_mode="line", evaluate_expression=None, print_expression=None, inspect_variable=None):
    # Compile the source code
    compiled_code = compile(source_code, '<string>', 'exec')

    # Set up the debugger
    debugger = pdb.Pdb()

    # Set breakpoints if specified
    if breakpoints:
        for line in breakpoints:
            debugger.set_break('<string>', line)

    # Perform the specified action
    if action == "set_breakpoint":
        return "Breakpoints set."
    elif action == "step":
        if step_mode == "line":
            debugger.run(compiled_code)
        elif step_mode == "return":
            debugger.runcall(compiled_code)
        elif step_mode == "continue":
            debugger.set_continue()
        elif step_mode == "next":
            debugger.set_next('<string>')
        return f"Stepping through the code in {step_mode} mode."
    elif action == "evaluate" and evaluate_expression:
        result = debugger.runeval(evaluate_expression)
        return f"Evaluating expression: {evaluate_expression} => {result}"
    elif action == "print" and print_expression:
        result = debugger.run(compiled_code)
        print_result = debugger.runsource(f"print({print_expression})")
        return f"Printed expression: {print_result}"
    elif action == "inspect" and inspect_variable:
        debugger.run(compiled_code)
        value = debugger.runsource(f"print({inspect_variable})")
        return f"Inspected variable: {value}"
    else:
        return "Action not supported."



# Main ast_tool function
def ast_tool(source_code, action, language, compile_mode="exec", test_class=None, target=None, options=None, output_format="string"):
    actions_map = {
        "compile": lambda: compile_code(source_code, compile_mode),
        "debug": lambda: debug_code(source_code),
        "optimize": lambda: optimize_code(source_code),
        "analyze": lambda: analyze_code(source_code, options),
        "test": lambda: test_code(test_class) if test_class else "Test class not provided.",
        "visualize": lambda: visualize_code(source_code),
        "unparse": lambda: unparse_ast(source_code)
    }
    action_func = actions_map.get(action)
    result = action_func() if action_func else "Action not supported."
    if output_format == "json":
        import json
        result = json.dumps(result, indent=4)
    return result
