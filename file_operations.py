import os
import shutil

# def file_operations(operation_type: str, source_path: str, destination_path: str = None, content: str = None):
#     """
#     Performs various file operations such as read, write, copy, and move.

#     Parameters:
#     - operation_type: The type of file operation ('read', 'write', 'copy', 'move')
#     - source_path: The source path; could be a file or directory.
#     - destination_path: The destination path; could be a file or directory (optional).
#     - content: The content to write to the file (optional, only for 'write').

#     Returns:
#     - A string containing the file content for 'read', or a confirmation message for other operations.
#     """
#     base_path = '/home/stahlubuntu/coder_agent/bd/'
#     source_full_path = os.path.join(base_path, source_path)
    
#     if operation_type == 'read':
#         if os.path.isdir(source_full_path):
#             return ', '.join(os.listdir(source_full_path))
#         else:
#             with open(source_full_path, 'r') as f:
#                 return f.read()

#     elif operation_type == 'write':
#         destination_full_path = os.path.join(base_path, destination_path) if destination_path else source_full_path
#         with open(destination_full_path, 'w') as f:
#             f.write(content)
#         return f"Content written to {destination_full_path}"

#     elif operation_type == 'copy':
#         destination_full_path = os.path.join(base_path, destination_path)
#         shutil.copy(source_full_path, destination_full_path)
#         return f"File copied from {source_full_path} to {destination_full_path}"

#     elif operation_type == 'move':
#         destination_full_path = os.path.join(base_path, destination_path)
#         shutil.move(source_full_path, destination_full_path)
#         return f"File moved from {source_full_path} to {destination_full_path}"

#     else:
#         return "Invalid operation_type. Supported operations are 'read', 'write', 'copy', 'move'."



import os
import shutil

def file_operations(operation_type: str, source_path: str, destination_path: str = None, content: str = None):
    """
    Performs various file operations such as read, write, copy, move, and mkdir.

    Parameters:
    - operation_type: The type of file operation ('read', 'write', 'copy', 'move', 'mkdir')
    - source_path: The source file or directory path
    - destination_path: The destination file or directory path (optional)
    - content: The content to write to the file (optional, only for 'write')

    Returns:
    - A string containing the file content for 'read', or a confirmation message for other operations.
    """
    base_path = '/home/stahlubuntu/coder_agent/bd/'
    source_full_path = os.path.join(base_path, source_path)
    
    if operation_type == 'read':
        if os.path.isdir(source_full_path):
            return ', '.join(os.listdir(source_full_path))
        else:
            with open(source_full_path, 'r') as f:
                return f.read()

    elif operation_type == 'write':
        destination_full_path = os.path.join(base_path, destination_path) if destination_path else source_full_path
        with open(destination_full_path, 'w') as f:
            f.write(content)
        return f"Content written to {os.path.relpath(destination_full_path, base_path)}"

    elif operation_type == 'copy':
        destination_full_path = os.path.join(base_path, destination_path)
        shutil.copy(source_full_path, destination_full_path)
        return f"File copied from {os.path.relpath(source_full_path, base_path)} to {os.path.relpath(destination_full_path, base_path)}"

    elif operation_type == 'move':
        destination_full_path = os.path.join(base_path, destination_path)
        shutil.move(source_full_path, destination_full_path)
        return f"File moved from {os.path.relpath(source_full_path, base_path)} to {os.path.relpath(destination_full_path, base_path)}"

    elif operation_type == 'mkdir':
        destination_full_path = os.path.join(base_path, destination_path)
        os.makedirs(destination_full_path, exist_ok=True)
        return f"Directory {os.path.relpath(destination_full_path, base_path)} created."

    else:
        return "Invalid operation_type. Supported operations are 'read', 'write', 'copy', 'move', 'mkdir'."


# print(file_operations('read', ''))  # Should read the file
# print(file_operations('write', 'new_file.txt', content='Hello World'))  # Should create and write to new_file.txt
# print(file_operations('copy', 'new_file.txt', 'new_file_copy.txt'))  # Should copy new_file.txt to new_file_copy.txt
# print(file_operations('move', 'new_file_copy.txt', 'new_file_moved.txt'))  # Should move new_file_copy.txt to new_file_moved.txt
# print(file_operations('mkdir', 'destination_path', 'new_folder'))  # Should create a new directory called new_folder
