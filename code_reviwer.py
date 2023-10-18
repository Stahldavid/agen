import subprocess



# Changing the function name to `automated_code_reviewer` as requested, while keeping the string output format.

import subprocess

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


# test_code = "controller_manager.py"
# #Example usage: (Note: As the function is not actually running here, you won't see the expected output.)
# #Please run this function in your local Python environment to see the results.
# feedback = automated_code_reviewer(test_code)
# print(feedback)


