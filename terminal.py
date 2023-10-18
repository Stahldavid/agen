

import subprocess

# def terminal_access(command):
#     try:
#         # Set the folder path where the command will be executed
#         folder_path = "/home/stahlubuntu/coder_agent/bd/"
        
#         # Run the command in the specified folder path using bash shell
#         result = subprocess.run(["bash", "-c", command], capture_output=True, text=True, cwd=folder_path)
        
#         # If the command was successful, return the standard output
#         if result.returncode == 0:
#             return str(result.stdout)
#         # If the command failed, return the standard error
#         else:
#             return str(result.stderr)
    
#     # If an exception occurs, return the error message
#     except Exception as e:
#         return str(e)






# def terminal_access(command):
#     try:
#         # Set the folder path where the command will be executed
#         folder_path = "/home/stahlubuntu/coder_agent/bd/"
        
#         # Run the command in the specified folder path using bash shell
#         result = subprocess.run(["bash", "-c", command], capture_output=True, text=True, cwd=folder_path)
        
#         # If the command was successful, return the last 2000 characters of the standard output
#         if result.returncode == 0:
#             return str(result.stdout)[-2000:]
#         # If the command failed, return the last 2000 characters of the standard error
#         else:
#             return str(result.stderr)[-2000:]
    
#     # If an exception occurs, return the error message
#     except Exception as e:
#         return str(e)



def terminal_access(command):
    try:
        # Set the folder path where the command will be executed
        folder_path = "/home/stahlubuntu/coder_agent/bd/"
        
        # Run the command in the specified folder path using bash shell
        result = subprocess.run(["bash", "-c", command], capture_output=True, text=True, cwd=folder_path)
        
        # If the command was successful, return the first 1000 characters and the last 2000 characters of the standard output
        if result.returncode == 0:
            output = str(result.stdout)
            return f"First 1000 characters: {output[:1000]}\nFinal 2000 characters: {output[-2000:]}"
        # If the command failed, return the first 1000 characters and the last 2000 characters of the standard error
        else:
            error = str(result.stderr)
            return f"First 1000 characters: {error[:1000]}\nFinal 2000 characters: {error[-2000:]}"
    
    # If an exception occurs, return the error message
    except Exception as e:
        return str(e)


# command = "source /opt/ros/foxy/setup.bash"
# print(terminal_access(command))