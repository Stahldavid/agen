import cProfile
import pstats
import io
import os

# Constant folder path for code files
FOLDER_PATH = "/home/stahlubuntu/coder_agent/bd/"

def code_profiler(code_or_filename: str, sort_by: str = 'cumulative', limit: int = 10):
    """
    Profile the given Python code using cProfile and returns the profiling data sorted by the given criteria.
    
    Parameters:
    - code_or_filename: The Python code or filename to be profiled as a string.
    - sort_by: The criteria for sorting the profiling data (default is 'cumulative').
    - limit: The number of lines to limit the output to (default is 10).
    
    Returns:
    - A string containing the profiling data.
    """
    # If code_or_filename is a filename, read the file contents
    if code_or_filename.endswith('.py'):
        try:
            with open(os.path.join(FOLDER_PATH, code_or_filename), 'r') as f:
                code = f.read()
        except FileNotFoundError:
            raise ValueError('File not found')
    else:
        code = code_or_filename
    
    # Initialize the profiler
    profiler = cProfile.Profile()
    
    # Run the profiler
    profiler.enable()
    exec(code)
    profiler.disable()
    
    # Capture the profiling stats
    s = io.StringIO()
    ps = pstats.Stats(profiler, stream=s).sort_stats(sort_by)
    ps.print_stats(limit)
    
    # Return the captured string
    return s.getvalue()


# # Test code for profiling
# test_code = """
# for i in range(1000):
#     x = i * i
# """

# # Test the code_profiler function
# profile_output = code_profiler(test_code)
# print(profile_output)
