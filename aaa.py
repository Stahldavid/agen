import ast
import json

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


import ast
import json

# ... (ast_tool function definition here)

test_code1 = """
import math
x = math.sqrt(4)
y = x + 2
if y > 2:
    print("Hello")
"""
print("Test 1 Output:")
print(ast_tool(test_code1))
print("-----------")

# Test 2: Class, method, and function definitions
test_code2 = """
class MyClass:
    def my_method(self, arg1):
        return arg1 * 2
        
def my_function(arg2):
    return arg2 + 1

x = MyClass()
y = x.my_method(2)
z = my_function(y)
"""
print("Test 2 Output:")
print(ast_tool(test_code2))
print("-----------")

# Test 3: Exception handling
test_code3 = """
try:
    x = 1 / 0
except ZeroDivisionError:
    x = 0
finally:
    print("Done")
"""
print("Test 3 Output:")
print(ast_tool(test_code3))
print("-----------")

# Test 4: Loops and control structures
test_code4 = """
for i in range(3):
    if i > 1:
        print(i)
while True:
    break
"""
print("Test 4 Output:")
print(ast_tool(test_code4))
print("-----------")

# Test 5: Nested loops and control structures
test_code5 = """
for i in range(3):
    for j in range(2):
        if i > j:
            print(i, j)
"""
print("Test 5 Output:")
print(ast_tool(test_code5))
print("-----------")

# Test 6: Multiple imports and aliasing
test_code6 = """
import math
import sys
from datetime import datetime as dt
"""
print("Test 6 Output:")
print(ast_tool(test_code6))
print("-----------")

# Test 7: List comprehensions and lambda functions
test_code7 = """
squares = [x*x for x in range(4)]
add_one = lambda x: x + 1
"""
print("Test 7 Output:")
print(ast_tool(test_code7))
print("-----------")

# Test 8: Using built-in functions and methods
test_code8 = """
my_list = [1, 2, 3]
length = len(my_list)
my_list.append(4)
"""
print("Test 8 Output:")
print(ast_tool(test_code8))
print("-----------")
