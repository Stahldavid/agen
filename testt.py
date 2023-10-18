# Importing necessary modules for creating and testing the function
import unittest
import importlib.util
from io import StringIO
import sys
import jsonschema
from jsonschema import validate


# Refining the unit_test_runner function to execute and find test cases directly from the global scope

def unit_test_runner(code_or_filename, test_code_or_filename):
    # Redirect stdout to capture test results
    output = StringIO()
    sys.stdout = output
    
    # Create a global scope dictionary to store the loaded modules and variables
    global_scope = {}
    
    # Load the main code
    if code_or_filename.endswith('.py'):
        spec = importlib.util.spec_from_file_location("main_module", code_or_filename)
        main_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(main_module)
        global_scope.update(main_module.__dict__)
    else:
        exec(code_or_filename, global_scope)
        
    # Load the test code
    if test_code_or_filename.endswith('.py'):
        spec = importlib.util.spec_from_file_location("test_module", test_code_or_filename)
        test_module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(test_module)
        global_scope.update(test_module.__dict__)
    else:
        exec(test_code_or_filename, global_scope)
    
    # Collect test cases from the global scope
    test_cases = [obj for name, obj in global_scope.items() if isinstance(obj, type) and issubclass(obj, unittest.TestCase)]
    
    # Run the tests
    for test_case in test_cases:
        suite = unittest.TestLoader().loadTestsFromTestCase(test_case)
        unittest.TextTestRunner(stream=output).run(suite)
    
    # Reset stdout and return the captured output
    sys.stdout = sys.__stdout__
    return output.getvalue()


